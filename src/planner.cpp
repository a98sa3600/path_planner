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

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);

  if (Constants::startTopic) {
    subStart = n.subscribe("/current_pose", 1, &Planner::autoStart, this);
  } else {
    subStart = n.subscribe("/astar/initialpose", 1, &Planner::setStart, this);
  }  
  
};

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

  // plan if the switch is not set to manual and a transform is available
  if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    // assign the values to start from base_link
    start.pose.position.x = transform.getOrigin().x();
    start.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.orientation);

    if (grid->info.height >= start.pose.position.y && start.pose.position.y >= 0 &&
        grid->info.width >= start.pose.position.x && start.pose.position.x >= 0) {
      // set the start as valid and plan
      validStart = true;
    } else  {
      validStart = false;
    }

    plan();
  }
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

    if (Constants::manual) { plan();}

    // publish start for RViz
    pubStart.publish(start);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

void Planner::autoStart(const geometry_msgs::PoseStamped::ConstPtr& initial) {
  if(validGoal){
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

      if (Constants::manual) { plan();}

      // publish start for RViz
      pubStart.publish(start);
    } else {
      std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
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

    if (Constants::manual) { plan();}

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
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
    // smoother.smoothPath(voronoiDiagram);
    // // CREATE THE UPDATED PATH
    // path.updatePath(smoother.getPath());

    // CREATE THE UPDATED PATH 
    std::vector<Node3D> nodes = smoother.getPath();

    // UPDATE VISTED COST AND REMAP XY TO REAL_WORLD 
    int tmp = -1;
    for (int i = 0; i < nodes.size(); i++){
      int idx = (int)(nodes[i].getY())*width+(int)(nodes[i].getX());
      if(tmp == idx){
        continue;
      }
      costMap[idx].setV(1);  
      tmp = idx;

    }

    createWayPoint(nodes,origin_x,origin_y);

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
    delete [] nodes3D;
    delete [] nodes2D;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}


    
void Planner::createWayPoint(std::vector<Node3D>& paths,float& origin_x,float& origin_y){
    autoware_msgs::LaneArray lane_array;
    autoware_msgs::Lane lane;
    std::vector<autoware_msgs::Waypoint> wps;
    // std::vector<Node3D> nodes = paths;
    int count = 0;
    for (int i = 0; i < paths.size(); i++){
        count+=1;
        paths[i].setX(origin_x + (paths[i].getX()*Constants::cellSize));
        paths[i].setY(origin_y + (paths[i].getY()*Constants::cellSize));
        autoware_msgs::Waypoint wp;
        wp.pose.pose.position.x = paths[i].getX();
        wp.pose.pose.position.y = paths[i].getY();
        wp.pose.pose.position.z = -3893.38; //Setting it wrong may cause problem publishing /safety_waypoint in the Astar_avoid.
        wp.twist.twist.linear.x = Helper::kmph2mps(10);
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
    lane.header.frame_id = "map";
    lane.header.stamp = ros::Time::now();
    
    lane.waypoints = wps;
    lane_array.lanes.emplace_back(lane);
    lane_pub_.publish(lane_array);
}