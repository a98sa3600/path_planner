#include "planner.h"

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;
  // _________________
  // TOPICS TO PUBLISH
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);
  lane_pub_ = n.advertise<autoware_msgs::LaneArray>("/based/lane_waypoints_raw", 10, true);

  // ___________________
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
  }

  subGoal_manual = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart_manual = n.subscribe("/astar/initialpose", 1, &Planner::setStart, this);
  subGoal_auto = n.subscribe("/target_pose/goal", 1, &Planner::autoGoal, this);
  subStart_auto = n.subscribe("/current_pose", 1, &Planner::autoStart, this);
  subDrive_out = n.subscribe("/drive_out",1,&Planner::setOut, this);
  n.param<bool>("hole15_switch", hole15_switch , false);
   
};

void parseWaypointForVer2(const std::string& line, autoware_msgs::Waypoint* wp)
{
  std::vector<std::string> columns;
  std::istringstream ss(line);
  std::string column;
  while (std::getline(ss, column, ',')){
    while (1){
      auto res = std::find(column.begin(), column.end(), ' ');
      if (res == column.end()){
        break;
      }
      column.erase(res);
    }
    if (!column.empty()){
      columns.emplace_back(column);
    }
  }
  wp->pose.pose.position.x = std::stof(columns[0]);
  wp->pose.pose.position.y = std::stof(columns[1]);
  wp->pose.pose.position.z = std::stof(columns[2]);
  wp->pose.pose.orientation = tf::createQuaternionMsgFromYaw(std::stof(columns[3]));
  wp->twist.twist.linear.x = Helper::kmph2mps(std::stoi(columns[4]));
  wp->change_flag = std::stoi(columns[5]);
  wp->wpstate.steering_state = 0;
  wp->wpstate.accel_state = 0;
  wp->wpstate.stop_state  = 0;
  wp->wpstate.event_state = 0;
}

void loadWaypointsForVer2(std::string& filename, std::vector<autoware_msgs::Waypoint>* csv_wps){
  std::ifstream ifs(filename);

	if(!ifs){
		std::cout << "Error File " << std::endl;
		return;
	}else{
    std::cout << filename << std::endl;
  }

  std::string line;
  std::getline(ifs, line);  // Remove first line
  
  while (std::getline(ifs, line)){
    autoware_msgs::Waypoint wp;
    parseWaypointForVer2(line, &wp);
    csv_wps->emplace_back(wp);
  }
}


//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

//###################################################
//                                                MAP
//###################################################
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  grid = map;
  //update the configuration space with the current map
  configurationSpace.updateGrid(map);
  //create array for Voronoi diagram
//  ros::Time t0 = ros::Time::now();
  int height = map->info.height;
  int width = map->info.width;
  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }

  costMap = new Map[width*height]();
  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
//  ros::Time t1 = ros::Time::now();
//  ros::Duration d(t1 - t0);
//  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  // // plan if the switch is not set to manual and a transform is available
  // if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

  //   listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

  //   // assign the values to start from base_link
  //   start.pose.position.x = transform.getOrigin().x();
  //   start.pose.position.y = transform.getOrigin().y();
  //   tf::quaternionTFToMsg(transform.getRotation(), start.pose.orientation);

  //   if (grid->info.height >= start.pose.position.y && start.pose.position.y >= 0 &&
  //       grid->info.width >= start.pose.position.x && start.pose.position.x >= 0) {
  //     // set the start as valid and plan
  //     validStart = true;
  //   } else  {
  //     validStart = false;
  //   }

  //   plan();
  // }
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  //retrieving start position
  float x = initial->pose.pose.position.x;
  float y = initial->pose.pose.position.y;
  float t = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  Eigen::VectorXd v(2);
  v << x-grid->info.origin.position.x, y-grid->info.origin.position.y;

  if ( v(0) <= (grid->info.width)*Constants::cellSize && v(0) >= 0 && 
        v(1) <= (grid->info.height)*Constants::cellSize && v(1) >= 0) {
    validStart = true;
    start = startN;

    check_to_plan();

    // publish start for RViz
    pubStart.publish(start);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

void Planner::autoStart(const geometry_msgs::PoseStamped::ConstPtr& initial) {
  if(validGoal){
    
    if(!drive_in){
      //retrieving start position
      float x = initial->pose.position.x;
      float y = initial->pose.position.y;
      float t = tf::getYaw(initial->pose.orientation);

      std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
      Eigen::VectorXd v(2);
      v << x-grid->info.origin.position.x, y-grid->info.origin.position.y;

      if (v(0) <= (grid->info.width)*Constants::cellSize && v(0) >= 0 && 
            v(1) <= (grid->info.height)*Constants::cellSize && v(1) >= 0) {
        validStart = true;
        start = *initial;   

        check_to_plan();

        // publish start for RViz
        pubStart.publish(start);
      }else {
        std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
      }
    }else{
      n.param<std::string>("hole_drive_in",waypoint_in_file_path,"hole1.csv" );
      std::cout << waypoint_in_file_path << std::endl;
      loadWaypointsForVer2(waypoint_in_file_path, &pre_waypoints_in);
      std::cout << pre_waypoints_in.size() << std::endl;
      int i = pre_waypoints_in.size()-1;
      float x = pre_waypoints_in.at(i).pose.pose.position.x;
      float y = pre_waypoints_in.at(i).pose.pose.position.y;
      float t = tf::getYaw(pre_waypoints_in.at(i).pose.pose.orientation);
      geometry_msgs::PoseStamped startN;
      startN.pose.position = pre_waypoints_in.at(i).pose.pose.position;
      startN.pose.orientation = pre_waypoints_in.at(i).pose.pose.orientation;
      startN.header.frame_id = "map";
      startN.header.stamp = ros::Time::now();
      pre_waypoints_in.pop_back();

      std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
      Eigen::VectorXd v(2);
      v << x-grid->info.origin.position.x, y-grid->info.origin.position.y;

      if (v(0) <= (grid->info.width)*Constants::cellSize && v(0) >= 0 && 
            v(1) <= (grid->info.height)*Constants::cellSize && v(1) >= 0) {
        validStart = true;
        start = startN;

        check_to_plan();

        // publish start for RViz
        pubStart.publish(start);
      }else {
        std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
      }      
    }

  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
  // retrieving goal position
  float x = end->pose.position.x;
  float y = end->pose.position.y;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  Eigen::VectorXd v(2);
  v << x-grid->info.origin.position.x, y-grid->info.origin.position.y;

  if (v(0) <= (grid->info.width)*Constants::cellSize && v(0) >= 0 && 
        v(1) <= (grid->info.height)*Constants::cellSize && v(1) >= 0) {
    validGoal = true;
    goal = *end;

    check_to_plan();

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}
void Planner::autoGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
  // retrieving goal position
  float x = end->pose.position.x;
  float y = end->pose.position.y;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  Eigen::VectorXd v(2);
  v << x-grid->info.origin.position.x, y-grid->info.origin.position.y;

  if (v(0) <= (grid->info.width)*Constants::cellSize && v(0) >= 0 && 
        v(1) <= (grid->info.height)*Constants::cellSize && v(1) >= 0) {
    validGoal = true;
    goal = *end;

    check_to_plan();

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                          DRIVE OUT 
//###################################################
void Planner::setOut(const std_msgs::Bool::ConstPtr& out){
  drive_out = out->data;
  if(drive_out){
    n.param<std::string>("hole_drive_out",waypoint_out_file_path,"hole1.csv" );
    std::cout << waypoint_out_file_path << std::endl;
    loadWaypointsForVer2(waypoint_out_file_path, &pre_waypoints_out);
    float x = pre_waypoints_out.at(0).pose.pose.position.x;
    float y = pre_waypoints_out.at(0).pose.pose.position.y;
    float t = tf::getYaw(pre_waypoints_out.at(0).pose.pose.orientation);
    geometry_msgs::PoseStamped end;
    end.pose.position = pre_waypoints_out.at(0).pose.pose.position;
    end.pose.orientation = pre_waypoints_out.at(0).pose.pose.orientation;
    end.header.frame_id = "map";
    end.header.stamp = ros::Time::now();

    std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
    Eigen::VectorXd v(2);
    v << x-grid->info.origin.position.x, y-grid->info.origin.position.y;

    if (v(0) <= (grid->info.width)*Constants::cellSize && v(0) >= 0 && 
          v(1) <= (grid->info.height)*Constants::cellSize && v(1) >= 0) {
      validGoal = true;
      goal = end;
      check_to_plan();
    } else {
      std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
    }
  }
}

//###################################################
//                                      CHECK_TO_PLAN
//###################################################
void Planner::check_to_plan(){
  std::cout << "check_to_plan start!!!" << std::endl;
  if(hole15_switch && validStart && validGoal){
    std::cout << "hole15_switch !!!" << std::endl;
    float start_tmp = Constants::hole15_profile_a * start.pose.position.x + Constants::hole15_profile_b;
    float goal_tmp = Constants::hole15_profile_a * goal.pose.position.x + Constants::hole15_profile_b;
    if( (start.pose.position.y >= start_tmp) && (goal.pose.position.y <= goal_tmp) ){
      std::cout << "hole15_bridge start!!!" << std::endl;
      hole15_bridge = true;
      hole15_case_plan();
    }else{
      hole15_bridge = false;
      if (Constants::manual) { plan();}
    }
  }
  else{   
    if (Constants::manual) { plan();}
  }

}

//###################################################
//                              SPECIAL_CASE(HOLE 15)
//###################################################
void Planner::hole15_case_plan(){
  std::cout << "hole15_case_plan start!!!" << std::endl;
  geometry_msgs::PoseStamped start_tmp = start;
  geometry_msgs::PoseStamped goal_tmp = goal;
  if(hole15_bridge_count){
    n.param<std::string>("hole15_brige_big",waypoint_big_bridge_file_path,"hole15_brige_big.csv" );
    std::cout << waypoint_big_bridge_file_path << std::endl;
    loadWaypointsForVer2(waypoint_big_bridge_file_path, &pre_waypoints_bridge);
    hole15_bridge_count++;
    if(hole15_bridge_count == 2) {hole15_bridge_count=0;}
  }else{
    n.param<std::string>("hole15_brige_small",waypoint_small_bridge_file_path,"hole15_brige_small.csv" );
    std::cout << waypoint_small_bridge_file_path << std::endl;
    loadWaypointsForVer2(waypoint_small_bridge_file_path, &pre_waypoints_bridge);
    hole15_bridge_count++;
  }

  // create first_part_waypoints
  std::cout << "first_part_waypoints start!!!" << std::endl;
  geometry_msgs::PoseStamped end;
  end.pose.position = pre_waypoints_bridge.at(0).pose.pose.position;
  end.pose.orientation = pre_waypoints_bridge.at(0).pose.pose.orientation;
  end.header.frame_id = "map";
  end.header.stamp = ros::Time::now();
  start = start_tmp;
  goal = end;
  if (Constants::manual) { plan();}
  first_part_waypoints = wps;
  wps.clear();

  // create seconds_part_waypoints
  std::cout << "seconds_part_waypoints start!!!" << std::endl;
  validStart = true;
  validGoal = true;
  geometry_msgs::PoseStamped startN;
  startN.pose.position = pre_waypoints_bridge.at(pre_waypoints_bridge.size()-1).pose.pose.position;
  startN.pose.orientation = pre_waypoints_bridge.at(pre_waypoints_bridge.size()-1).pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();
  pre_waypoints_bridge.pop_back();
  start = startN;
  goal = goal_tmp;
  if (Constants::manual) { plan();}
  second_part_waypoints = wps;
  wps.clear();


  // publish waypoints
  std::cout << "publish_waypoints start!!!" << std::endl;
  autoware_msgs::LaneArray lane_array;
  autoware_msgs::Lane lane;
  if(drive_in){
    pre_waypoints_in.insert(pre_waypoints_in.end(),first_part_waypoints.begin(),first_part_waypoints.end());
    pre_waypoints_in.insert(pre_waypoints_in.end(),pre_waypoints_bridge.begin(),pre_waypoints_bridge.end());
    pre_waypoints_in.insert(pre_waypoints_in.end(),second_part_waypoints.begin(),second_part_waypoints.end());
    lane.waypoints = pre_waypoints_in;
    pre_waypoints_in.clear();
    first_part_waypoints.clear();
    pre_waypoints_bridge.clear();
    second_part_waypoints.clear();
    drive_in = false;
  }else if(drive_out){
    first_part_waypoints.insert(first_part_waypoints.end(),pre_waypoints_bridge.begin(),pre_waypoints_bridge.end());
    first_part_waypoints.insert(first_part_waypoints.end(),second_part_waypoints.begin(),second_part_waypoints.end());
    first_part_waypoints.insert(first_part_waypoints.end(),pre_waypoints_out.begin(),pre_waypoints_out.end());
    lane.waypoints = first_part_waypoints;
    first_part_waypoints.clear();
    pre_waypoints_bridge.clear();
    second_part_waypoints.clear();
    pre_waypoints_out.clear();
    drive_out = false;      
  }else{
    first_part_waypoints.insert(first_part_waypoints.end(),pre_waypoints_bridge.begin(),pre_waypoints_bridge.end());
    first_part_waypoints.insert(first_part_waypoints.end(),second_part_waypoints.begin(),second_part_waypoints.end());
    lane.waypoints = first_part_waypoints;
    first_part_waypoints.clear();
    pre_waypoints_bridge.clear();
    second_part_waypoints.clear();
  }
  lane.header.frame_id = "map";
  lane.header.stamp = ros::Time::now();
  lane_array.lanes.emplace_back(lane);
  lane_pub_.publish(lane_array);

}
//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::plan() {
  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal) {

    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();
    
    float origin_x = grid->info.origin.position.x;
    float origin_y = grid->info.origin.position.y;

    // _________________________
    // retrieving start position
    float x = (start.pose.position.x-origin_x) / Constants::cellSize;
    float y = (start.pose.position.y-origin_y) / Constants::cellSize;
    float t = tf::getYaw(start.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, 0, nullptr);
    std::cout << "start: " << x << ", " << y << ", "<< t << std::endl;
    // ___________
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);  

    // ________________________
    // retrieving goal position
    x = (goal.pose.position.x-origin_x) / Constants::cellSize;
    y = (goal.pose.position.y-origin_y) / Constants::cellSize;
    t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, 0, nullptr);
    std::cout << "goal: " << x << ", " << y << ", "<< t << std::endl;
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);


    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // // CLEAR THE PATH
    // path.clear();
    // smoothedPath.clear();

    // FIND THE PATH
    Node3D*  nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization,costMap);
    
    // TRACE THE PATH
    smoother.tracePath(nSolution);
    smoother.smoothPath(voronoiDiagram);
    // // CREATE THE UPDATED PATH
    // path.updatePath(smoother.getPath());

    // CREATE THE UPDATED PATH 
    std::vector<Node3D> nodes = smoother.getPath();

    // UPDATE VISTED COST AND REMAP XY TO REAL_WORLD 
    int tmp = -1;
    for (size_t i = 0; i < nodes.size(); i++){
      int idx = (int)(nodes[i].getY())*width+(int)(nodes[i].getX());
      if(tmp == idx){
        continue;
      }
      costMap[idx].setV(1);  
      tmp = idx;
    }

    createWayPoint(nodes);

    if(!hole15_bridge){
      publishWayPoint();
    }
    // // _________________________________
    // // PUBLISH THE RESULTS OF THE SEARCH
    // path.updatePath(nodes);
    // path.publishPath();
    // path.publishPathNodes();
    // path.publishPathVehicles();

    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;

    validGoal = false ;
    validStart = false ;
    delete [] nodes3D;
    delete [] nodes2D;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}


    
void Planner::createWayPoint(std::vector<Node3D>& paths){
    wps.clear();
    size_t count = 0;
    for (size_t i = 0; i < paths.size(); i++){
        count+=1;
        paths[i].setX(grid->info.origin.position.x + (paths[i].getX()*Constants::cellSize));
        paths[i].setY(grid->info.origin.position.y + (paths[i].getY()*Constants::cellSize));
        autoware_msgs::Waypoint wp;
        wp.pose.pose.position.x = paths[i].getX();
        wp.pose.pose.position.y = paths[i].getY();
        wp.pose.pose.position.z = -3893.38; //Setting it wrong may cause problem publishing /safety_waypoint in the Astar_avoid.

        if(drive_out){
          if(  (((paths.size()-count) < 10)&& !drive_in)  ) {
              wp.twist.twist.linear.x = Helper::kmph2mps(5);
          }else if (  (((paths.size()-count) < 15)&& !drive_in) ) {
              wp.twist.twist.linear.x = Helper::kmph2mps(7);
          }else {
              wp.twist.twist.linear.x = Helper::kmph2mps(10);
          }
        }else{
          if(  (((paths.size()-count) < 10)&& !drive_in) || (count<10 ) ){
              wp.twist.twist.linear.x = Helper::kmph2mps(5);
          }else if (  (((paths.size()-count) < 15)&& !drive_in) || (count<15) ){
              wp.twist.twist.linear.x = Helper::kmph2mps(7);
          }else{
              wp.twist.twist.linear.x = Helper::kmph2mps(10);
          }
        }
        // wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(paths[i].getT() );
        wp.change_flag = 0;
        wp.wpstate.steering_state = 0;
        wp.wpstate.accel_state = 0;
        wp.wpstate.stop_state  = 0;
        wp.wpstate.event_state = 0;
        wps.emplace_back(wp);
    }

    std::reverse(wps.begin(), wps.end());
    size_t last = count - 1;
    for (size_t i = 0; i < wps.size(); ++i)
    {
        if (i != last){
        double yaw = atan2(wps.at(i + 1).pose.pose.position.y - wps.at(i).pose.pose.position.y,
                            wps.at(i + 1).pose.pose.position.x - wps.at(i).pose.pose.position.x);
        wps.at(i).pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
        else{
        wps.at(i).pose.pose.orientation = wps.at(i - 1).pose.pose.orientation;
        }
    }

}

void Planner::publishWayPoint(){
    autoware_msgs::LaneArray lane_array;
    autoware_msgs::Lane lane;

    if(drive_in){
      for (size_t i = 0; i < wps.size(); ++i)
      {
        pre_waypoints_in.emplace_back(wps.at(i));
      }      
      lane.waypoints = pre_waypoints_in;
      pre_waypoints_in.clear();
      drive_in = false;
    }else if(drive_out){
      for (size_t i = 0 ; i < pre_waypoints_out.size(); ++i)
      {
        wps.emplace_back(pre_waypoints_out.at(i));
      }      
      lane.waypoints = wps;
      pre_waypoints_out.clear();
      drive_out = false;      
    }else{
      lane.waypoints = wps;
    }
    lane.header.frame_id = "map";
    lane.header.stamp = ros::Time::now();
    lane_array.lanes.emplace_back(lane);
    lane_pub_.publish(lane_array);
}
