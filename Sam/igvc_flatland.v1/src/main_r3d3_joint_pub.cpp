// ROS and node class header file
#include <ros/ros.h>
#include "r3d3JointPub.h"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "r3d3_joint_pub");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  r3d3_model::r3d3JointPub node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
