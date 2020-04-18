/*#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_broadcaster.h>
#include <gps_sim_project/MarkerConfig.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <ugv_course_libs/gps_conv.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <igvc_flatland/WaypointList.h>
#include <sensor_msgs/LaserScan.h>


//final_project::MarkerConfig config;


std::vector<double> waypoint_lat;
std::vector<double> waypoint_lon;

std::vector<tf::Vector3> local_waypoints;


UTMCoords ref_coords;
tf::Vector3 relative_position;
nav_msgs::Path gps_path;

ros::Publisher pub_path;
ros::Publisher pub_cmd_vel;
ros::Publisher pub_waypoint;
ros::Publisher pub_markers;
ros::Publisher pub_lidar;
ros::Publisher pub_camera;
ros::Publisher pub_String_Left;
ros::Publisher pub_String_Right;
const double WHEEL_DISTANCE = 1;
const double WHEEL_RADIUS = 0.2;


void recvTwist(const geometry_msgs::TwistStamped::ConstPtr& msg){
    double v = msg->twist.linear.x;
    double psi_dot = msg->twist.angular.z;
    
    std_msgs::Float64 left_wheel_speed;
    std_msgs::Float64 right_wheel_speed;
    
    left_wheel_speed.data = (1/WHEEL_RADIUS)*(v - WHEEL_DISTANCE*psi_dot/2);
    right_wheel_speed.data = (1/WHEEL_RADIUS)*(v + WHEEL_DISTANCE*psi_dot/2);
    
    pub_String_Left.publish(left_wheel_speed);
    pub_String_Right.publish(right_wheel_speed);
    
    /*geometry_msgs::Twist cmd_vel;   
       
    cmd_vel.linear.x = v;
    cmd_vel.angular.z = psi_dot;

    pub_cmd_vel.publish(cmd_vel);
    
    v=0.0;
    psi_dot=-0.0;

} 

void recvLidar(const sensor_msgs::LaserScan::ConstPtr& msg){
    
} 

void recvCamera(const sensor_msgs::LaserScan::ConstPtr& msg){
    
} 

void recvWaypoint(const igvc_flatland::WaypointList::ConstPtr& msg){
    
} 

void recvMarker(const visualization_msgs::MarkerArray::ConstPtr& msg){
    
}

/*void reconfig(final_project::MarkerConfig& cfg, const uint32_t &level){
    config = cfg;
} 

void recvFix(const sensor_msgs::NavSatFixConstPtr& msg){

}

void recvcallback(const std_msgs::Float64ConstPtr& msg){
    }     
    
void timerCallback(const ros::TimerEvent& event){    
    static tf::TransformBroadcaster broadcaster;
}

int main(int argc, char** argv){
    ros::init(argc,argv,"final_project");
    ros::NodeHandle nh;
    
    ros::Subscriber gps_sub = nh.subscribe("/gps/fix",1,recvFix);
    ros::Timer timer = nh.createTimer(ros::Duration(0.5), timerCallback);
    
    pub_path = nh.advertise<nav_msgs::Path>("gps_path", 1);
    
    double ref_lat;
    double ref_lon;
    nh.getParam("/audibot/gps/ref_lat",ref_lat);
    nh.getParam("/audibot/gps/ref_lon",ref_lon);

    LatLon ref_coords_lat_lon(ref_lat,ref_lon,0);
    ref_coords = UTMCoords(ref_coords_lat_lon);
    double central_meridian = ref_coords.getCentralMeridian();
    ROS_INFO("Central Meridian of the Reference Coordinate: %f", central_meridian);
    
    ros::Subscriber waypoint_sub = nh.subscribe("/waypoints",1,recvWaypoint);
    
    ros::Subscriber marker_node = nh.subscribe("/waypoint_markers", 1,recvMarker);    
    
    pub_waypoint = nh.advertise<igvc_flatland::WaypointList>("waypoints", 1);
    
    pub_markers = nh.advertise<visualization_msgs::MarkerArray>("marker_array",1);
    ros::Timer marker_timer = nh.createTimer(ros::Duration(0.05), timerCallback);
    
    ros::Subscriber lidar_node = nh.subscribe("/scan", 1,recvLidar);   
    
    pub_lidar = 
    nh.advertise<sensor_msgs::LaserScan>("lidar_obstacles", 1);
    
    ros::Subscriber camera_node = nh.subscribe("/camera_scan", 1,recvCamera);
    
    pub_camera = 
    nh.advertise<sensor_msgs::LaserScan>("camera_lines", 1);
    
    ros::Subscriber twist_node = nh.subscribe("/twist", 1,recvTwist);       
  
    //pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 
        
    dynamic_reconfigure::Server<final_project::MarkerConfig> srv;
    srv.setCallback(boost::bind(reconfig,_1,_2));
    
     
    ros::Subscriber sub_twist = nh.subscribe("/cmd_vel", 1,recvTwist);       // Subscriber declaration
  
  pub_String_Left = nh.advertise<std_msgs::Float64>("/mantis/left_wheel_controller/command", 1);                 
  pub_String_Right = nh.advertise<std_msgs::Float64>("/mantis/right_wheel_controller/command", 1);                 // Publisher declaration
    
  ros::spin();
}
*/
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

int current_waypoint = 0;

move_base_msgs::MoveBaseGoal goal;

int main(int argc, char** argv){
ros::init(argc, argv, "simple_navigation_goals");

ROS_INFO("Start move base");

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//tell the action client that we want to spin a thread by default
MoveBaseClient ac("move_base", true);

//wait for the action server to come up
while(!ac.waitForServer(ros::Duration(5.0))){
ROS_INFO("Waiting for the move_base action server to come up");
}
 

if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ++current_waypoint; 
}

ROS_INFO("check waypoint");

    
switch(current_waypoint){
 case 0 :
     
ROS_INFO("current_waypoint check 1: (%d)", current_waypoint);


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//we'll send a goal to the robot to move 1 meter forward
goal.target_pose.header.frame_id = "base";
goal.target_pose.header.stamp = ros::Time::now();
 
goal.target_pose.pose.position.x = 1.0;
goal.target_pose.pose.position.y = 0.0;
goal.target_pose.pose.orientation.w = 1.0;
 
ROS_INFO("Sending goal");
ac.sendGoal(goal);

ac.waitForResult();
ROS_INFO("Waiting for result");

if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ++current_waypoint; 
    ROS_INFO("Hooray, the base moved 1 meter forward");
}
else
     ROS_INFO("The base failed to move forward 1 meter for some reason");

ROS_INFO("current_waypoint check 2: (%d)", current_waypoint);

ROS_INFO("end case 0");

 case 1 : 
     
ROS_INFO("start case 1");

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
     
//we'll send a goal to the robot to move 1 meter forward
goal.target_pose.header.frame_id = "base";
goal.target_pose.header.stamp = ros::Time::now();
 
goal.target_pose.pose.position.x = 2.0;
goal.target_pose.pose.position.y = 0.0;
goal.target_pose.pose.orientation.w = 1.0;

ROS_INFO("Sending goal");
ac.sendGoal(goal);
 
ac.waitForResult();

 case 2 : 
     
ROS_INFO("start case 2");

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
     
//we'll send a goal to the robot to move 1 meter forward
goal.target_pose.header.frame_id = "base";
goal.target_pose.header.stamp = ros::Time::now();
 
goal.target_pose.pose.position.x = 2.0;
goal.target_pose.pose.position.y = 0.0;
goal.target_pose.pose.orientation.w = 1.0;

ROS_INFO("Sending goal");
ac.sendGoal(goal);
 
ac.waitForResult();

 case 3 : 
     
ROS_INFO("start case 3");

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
     
//we'll send a goal to the robot to move 1 meter forward
goal.target_pose.header.frame_id = "base";
goal.target_pose.header.stamp = ros::Time::now();
 
goal.target_pose.pose.position.x = 2.0;
goal.target_pose.pose.position.y = 0.0;
goal.target_pose.pose.orientation.w = 1.0;

ROS_INFO("Sending goal");
ac.sendGoal(goal);
 
ac.waitForResult();

 case 4 : 
     
ROS_INFO("start case 4");

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
     
//we'll send a goal to the robot to move 1 meter forward
goal.target_pose.header.frame_id = "base";
goal.target_pose.header.stamp = ros::Time::now();
 
goal.target_pose.pose.position.x = 2.0;
goal.target_pose.pose.position.y = 0.0;
goal.target_pose.pose.orientation.w = 1.0;

ROS_INFO("Sending goal");
ac.sendGoal(goal);
 
ac.waitForResult();

 case 5 : 
     
ROS_INFO("start case 5");

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
     
//we'll send a goal to the robot to move 1 meter forward
goal.target_pose.header.frame_id = "base";
goal.target_pose.header.stamp = ros::Time::now();
 
goal.target_pose.pose.position.x = 1.0;
goal.target_pose.pose.position.y = 0.0;
goal.target_pose.pose.orientation.w = 1.0;

ROS_INFO("Sending goal");
ac.sendGoal(goal);
 
ac.waitForResult();

 case 6 : 
     
ROS_INFO("start case 6");

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
     
//we'll send a goal to the robot to move 1 meter forward
goal.target_pose.header.frame_id = "base";
goal.target_pose.header.stamp = ros::Time::now();
 
goal.target_pose.pose.position.x = 1.0;
goal.target_pose.pose.position.y = 0.0;
goal.target_pose.pose.orientation.w = 1.0;

ROS_INFO("Sending goal");
ac.sendGoal(goal);
 
ac.waitForResult();

 case 7 : 
     
ROS_INFO("start case 7");

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
     
//we'll send a goal to the robot to move 1 meter forward
goal.target_pose.header.frame_id = "base";
goal.target_pose.header.stamp = ros::Time::now();
 
goal.target_pose.pose.position.x = 1.0;
goal.target_pose.pose.position.y = 0.5;
goal.target_pose.pose.orientation.w = 1.0;

ROS_INFO("Sending goal");
ac.sendGoal(goal);
 
ac.waitForResult();

 case 8 : 
     
ROS_INFO("start case 8");

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
     
//we'll send a goal to the robot to move 1 meter forward
goal.target_pose.header.frame_id = "base";
goal.target_pose.header.stamp = ros::Time::now();
 
goal.target_pose.pose.position.x = 1.0;
goal.target_pose.pose.position.y = 0.0;
goal.target_pose.pose.orientation.w = 1.0;

ROS_INFO("Sending goal");
ac.sendGoal(goal);
 
ac.waitForResult();

 }

ros::spin();
}

