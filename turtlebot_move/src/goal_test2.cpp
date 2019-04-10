#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>


using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  int i =0;
  int numofcoords= 2;
  double coordinates[numofcoords][7] = { {3.61,3.26,0,0,0,0.99,0.008},
                                         {2.47,-0.14,0,0,0,-0.99,0.064} };

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

 while (true) { 
  
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  goal.target_pose.pose.position.x = coordinates[i][0];
  goal.target_pose.pose.position.y = coordinates[i][1];
  goal.target_pose.pose.position.z = coordinates[i][2] ;
  
  goal.target_pose.pose.orientation.x = coordinates[i][3];
  goal.target_pose.pose.orientation.y = coordinates[i][4];
  goal.target_pose.pose.orientation.z = coordinates[i][5];
  goal.target_pose.pose.orientation.w = coordinates[i][6];

  ROS_INFO("Sending goal");

 // ROS_INFO<< "best path cost: " << bestPath->getCost() << endl;
  ac.sendGoal(goal);
  
  ac.waitForResult();
   
  i++;

  if (i>= numofcoords) {
   
    i=0;
   }
  /*if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");*/
  }
  return 0;
}
