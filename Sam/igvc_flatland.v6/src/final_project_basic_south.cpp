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
goal.target_pose.header.frame_id = "map";
goal.target_pose.header.stamp = ros::Time::now();
 
goal.target_pose.pose.position.x = 22;
goal.target_pose.pose.position.y = -15.23;
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
     
ROS_INFO("start case 1: North Opening");

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
     
//we'll send a goal to the robot to move 1 meter forward
goal.target_pose.header.frame_id = "map";
goal.target_pose.header.stamp = ros::Time::now();
 
goal.target_pose.pose.position.x = 21.92;
goal.target_pose.pose.position.y = 15.65;
goal.target_pose.pose.orientation.w = 1.0;

ROS_INFO("Sending goal");
ac.sendGoal(goal);
 
ac.waitForResult();

 case 2 : 
     
ROS_INFO("start case 2: Finish");

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
     
//we'll send a goal to the robot to move 1 meter forward
goal.target_pose.header.frame_id = "map";
goal.target_pose.header.stamp = ros::Time::now();
 
goal.target_pose.pose.position.x = -25;
goal.target_pose.pose.position.y = -0;
goal.target_pose.pose.orientation.w = 1.0;

ROS_INFO("Sending goal");
ac.sendGoal(goal);
 
ac.waitForResult();

 }

ros::spin();
}

