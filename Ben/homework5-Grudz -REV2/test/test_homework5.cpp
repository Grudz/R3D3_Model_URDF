#include <ros/ros.h>
#include <gtest/gtest.h>
#include <stdio.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>

ros::NodeHandle* node;
bool mantis_alive = false;
bool result_received = false;
move_base_msgs::MoveBaseActionResult result;

ros::Publisher pub_goal;

void recvScan(const sensor_msgs::LaserScanConstPtr& msg)
{
  // Use simulated laser scan as indication that Mantis is loaded and ready to go
  mantis_alive = true;
}

void recvResult(const move_base_msgs::MoveBaseActionResultConstPtr& msg)
{
  result_received = true;
  result = *msg;
}

TEST(Homework5Test, goal_navigation_test) {
  while(!ros::isShuttingDown() && !mantis_alive){
    ros::Duration(0.02).sleep();
  }

  geometry_msgs::PoseStamped goal_msg;
  goal_msg.header.stamp = ros::Time::now();
  goal_msg.header.frame_id = "map";
  goal_msg.pose.position.x = 18.0;
  goal_msg.pose.position.y = 14.5;
  goal_msg.pose.orientation.w = 1.0;
  pub_goal.publish(goal_msg);

  while (!ros::isShuttingDown() && !result_received){
    ros::Duration(0.02).sleep();
  }

  ASSERT_EQ(result.status.status, 3);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_homework5");

  node = new ros::NodeHandle();

  ros::Subscriber sub_scan = node->subscribe("/mantis/laser/scan", 1, recvScan);
  ros::Subscriber sub_result = node->subscribe("/move_base/result", 1, recvResult);
  pub_goal = node->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

  ros::AsyncSpinner spinner(3);
  spinner.start();

  int result = RUN_ALL_TESTS();

  spinner.stop();
  node->shutdown();
  delete node;

  return 0;
}
