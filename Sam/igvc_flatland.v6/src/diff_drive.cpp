#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub_String_Left;
ros::Publisher pub_String_Right;
const double WHEEL_DISTANCE = 1;
const double WHEEL_RADIUS = 0.2;

void recvTwist(const geometry_msgs::TwistConstPtr& msg){
    double v = msg->linear.x;
    double psi_dot = msg->angular.z;

    std_msgs::Float64 left_wheel_speed;
    std_msgs::Float64 right_wheel_speed;
    
    left_wheel_speed.data = (1/WHEEL_RADIUS)*(v - WHEEL_DISTANCE*psi_dot/2);
    right_wheel_speed.data = (1/WHEEL_RADIUS)*(v + WHEEL_DISTANCE*psi_dot/2);
    
    pub_String_Left.publish(left_wheel_speed);
    pub_String_Right.publish(right_wheel_speed);
}       // Callback function and global variables

int main(int argc, char **argv)
{
  ros::init(argc, argv, "diff_drive");
  ros::NodeHandle nh;

  ros::Subscriber sub_twist = nh.subscribe("/cmd_vel", 1,recvTwist);       // Subscriber declaration
  
  pub_String_Left = nh.advertise<std_msgs::Float64>("/mantis/left_wheel_controller", 1);                 
  pub_String_Right = nh.advertise<std_msgs::Float64>("/mantis/right_wheel_controller", 1);                 // Publisher declaration
  
  ros::spin();
}
