
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


// Sign Picture defined by 7 x 7
uint8_t sign_row = 7;
uint8_t sign_column = 7;
uint8_t A_index[] = {9,10,11,16,18,23,24,25,30,32,37,39};
uint8_t B_index[] = {9,10,11,16,18,23,24,25,30,32,37,38,39};
uint8_t C_index[] = {9,10,11,16,18,23,30,32,37,38,39};
uint8_t D_index[] = {9,10,16,18,23,25,30,32,37,38};
uint8_t E_index[] = {9,10,11,16,23,24,25,30,37,38,39};
int A_origin[] = {94330,209137};
int B_origin[] = {94339,209168};
int C_origin[] = {94318,209182};
int D_origin[] = {94311,209215};
int E_origin[] = {94280,209234};
visualization_msgs::Marker points_red,points_green;

void addSign(int *origin,uint8_t *index){
    uint8_t i = 0;
    uint8_t red_flag=0;
    for(uint8_t x=0;x<sign_row;x++){
        for(uint8_t y=0;y<sign_column;y++){
            geometry_msgs::Point p;
            p.x = (int32_t)origin[0]+y;
            p.y = (int32_t)origin[1]-x;
            p.z = (int32_t)0;  
            if(red_flag == index[i] ){
                points_red.points.push_back(p);
                i++;
            }else{
                points_green.points.push_back(p);
            }
            red_flag++;
        }
    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "map_sign");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

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
    points_red.scale.x = points_green.scale.x = 1;
    points_red.scale.y = points_green.scale.y = 1;
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

    addSign(A_origin,A_index);
    addSign(B_origin,B_index);
    addSign(C_origin,C_index);
    addSign(D_origin,D_index);
    addSign(E_origin,E_index);


    marker_pub.publish(points_red);
    marker_pub.publish(points_green);
    r.sleep();
  }
}