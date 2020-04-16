// homework3 - Ben Grudzien
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <std_msgs/Float64.h>

ros::Publisher left_wheel_pub; 
ros::Publisher right_wheel_pub;
const double WHEEL_DISTANCE = 0.58; // drive_wheel_offset * 2 from mantis urdf
const double WHEEL_RADIUS = 0.165;

void recvTwist(const geometry_msgs::TwistConstPtr &msg){
    
    double v = msg->linear.x;
    double psi_dot = msg->angular.z;
    
    std_msgs::Float64 left_wheel_speed;
    std_msgs::Float64 right_wheel_speed;
    
    left_wheel_speed.data = (1/WHEEL_RADIUS)*(v - WHEEL_DISTANCE*psi_dot/2);
    right_wheel_speed.data = (1/WHEEL_RADIUS)*(v + WHEEL_DISTANCE*psi_dot/2);
    
    left_wheel_pub.publish(left_wheel_speed);
    right_wheel_pub.publish(right_wheel_speed);
    
}

int main(int argc, char** argv){
    
    ros::init(argc,argv,"diff_drive_mantis");
    ros::NodeHandle nh;
    
    ros::Subscriber sub_twist = nh.subscribe("/twist_cmd", 1, recvTwist);
    
    left_wheel_pub = nh.advertise<std_msgs::Float64>("/mantis/left_wheel_controller/command", 1);
    
    right_wheel_pub = nh.advertise<std_msgs::Float64>("/mantis/right_wheel_controller/command", 1);
    
    ros::spin();
}
