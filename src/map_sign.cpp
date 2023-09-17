
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <string> 
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include <algorithm>

// Sign Picture defined by 7 x 7
uint8_t sign_row = 7;
uint8_t sign_column = 7;
uint8_t A_index[] = {9,10,11,16,18,23,24,25,30,32,37,39};
uint8_t B_index[] = {9,10,11,16,18,23,24,25,30,32,37,38,39};
uint8_t C_index[] = {9,10,11,16,18,23,30,32,37,38,39};
uint8_t D_index[] = {9,10,16,18,23,25,30,32,37,38};
uint8_t E_index[] = {9,10,11,16,23,24,25,30,37,38,39};
uint8_t F_index[] = {9,10,11,16,23,24,25,30,37};
uint8_t G_index[] = {8,9,10,11,15,22,24,25,26,29,32,36,37,38,39};
uint8_t H_index[] = {9,11,16,18,23,24,25,30,32,37,39};
uint8_t I_index[] = {9,10,11,17,24,31,37,38,39};
uint8_t J_index[] = {9,10,11,17,24,31,37,38};
uint8_t K_index[] = {9,12,16,18,23,24,30,32,37,40};
uint8_t L_index[] = {9,16,23,30,37,38,39};
uint8_t M_index[] = {8,12,15,16,18,19,22,24,26,29,33,36,40};
uint8_t N_index[] = {8,12,15,16,19,22,24,26,29,32,33,};
uint8_t O_index[] = {9,10,11,16,18,23,25,30,32,37,38,39};
uint8_t P_index[] = {9,10,11,16,18,23,24,25,30,37};
uint8_t Q_index[] = {9,10,11,16,18,23,25,30,32,37,38,39,40};
uint8_t R_index[] = {9,10,11,16,18,23,24,25,30,31,37,39};
uint8_t S_index[] = {9,10,11,16,23,24,25,32,37,38,39};
uint8_t T_index[] = {9,10,11,17,24,31,38};



uint8_t *ALL_index[] = {A_index,B_index,C_index,D_index,E_index,F_index,G_index,H_index,
                      I_index,J_index,K_index,L_index,M_index,N_index,N_index,O_index,
                      R_index,S_index,T_index};

typedef struct point{
	int id;
	float x;
	float y;
}point_t;

visualization_msgs::Marker points_red,points_green;

void addSign(point_t &pt){
    uint8_t i = 0;
    uint8_t red_flag=0;
    uint8_t index_number = pt.id;
    float step_x =0.0;
    float step_y =0.0;
    for(uint8_t x=0;x<sign_row;x++){
        for(uint8_t y=0;y<sign_column;y++){
            geometry_msgs::Point p;
            p.x = (int32_t)pt.x+step_y;
            p.y = (int32_t)pt.y-step_x;
            p.z = (int32_t)-5000;           
            if(red_flag == ALL_index[index_number][i] ){
                points_red.points.push_back(p);
                i++;
            }else{
                points_green.points.push_back(p);
            }
            red_flag++;
            step_y+=0.5;
        }
      step_x+=0.5;
      step_y=0;
    }
}

void parseColumns(const std::string& line, std::vector<std::string>* columns){
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
      columns->emplace_back(column);
    }

  }
}

void parsePoint(const std::string& line, point_t* pt){
  std::vector<std::string> columns;
  parseColumns(line, &columns);
  pt->id = std::stoi(columns[0]);
  pt->x = std::stof(columns[1]);
  pt->y = std::stof(columns[2]);
}

void getHolePoints(std::string filePath,std::vector<point_t>* points){
	std::ifstream ifs(filePath);

	if(!ifs){
		std::cout << "Error File " << std::endl;
		return;
	}
	std::string line;
	std::getline(ifs,line); //Remove first line

	while (std::getline(ifs, line)){
		point_t pt;
		parsePoint(line, &pt);
		points->emplace_back(pt);
	}
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "map_sign");
    ros::NodeHandle nh("~");
    ros::Rate r(1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    std::string filePath;
    nh.param<std::string>("hole_map_sign",filePath,"hole1_map_sign.csv" );

    std::cout << "Loading file_path : " << filePath << std::endl;
    std::vector<point_t> points;
    getHolePoints(filePath, &points);

    while (ros::ok())
    { 
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        points_red.header.frame_id = points_green.header.frame_id = "map";
        points_red.header.stamp = points_green.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        points_red.ns = points_green.ns = "map_sign";
        // Set the marker type.  Initially this is points, and cycles between that and SPHERE, ARROW, and CYLINDER
        points_red.type = points_green.type = visualization_msgs::Marker::POINTS;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        points_red.action = points_green.action= visualization_msgs::Marker::ADD;
        points_red.pose.orientation.w = points_green.pose.orientation.w = 1.0;

        points_red.id = 0;
        points_green.id = 1;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        points_red.scale.x = points_green.scale.x = 0.5;
        points_red.scale.y = points_green.scale.y = 0.5;
        // points_red.scale.z = points_green.scale.z= 0.1;

        // Set the color -- be sure to set alpha to something non-zero!
        points_red.color.r = 1.0f;
        points_red.color.g = 0.0f;
        points_red.color.b = 0.0f;
        points_red.color.a = 1.0;  

        // Set the color -- be sure to set alpha to something non-zero!
        points_green.color.r = 0.0f;
        points_green.color.g = 1.0f;
        points_green.color.b = 0.0f;
        points_green.color.a = 1.0;  

        for (size_t i = 0; i < points.size(); ++i){
            point_t pt= points[i];
            addSign(pt);
        }

        marker_pub.publish(points_red);
        marker_pub.publish(points_green);
        r.sleep();
    }
}