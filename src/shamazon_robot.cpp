/**
 * BSD-3 Clause
 *
 * Copyright (c) 2021 Maaruf Vazifdar, Maitreya Kulkarni, Pratik Bhujbal
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to 
 * deal in the Software without restriction, including without limitation the  
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE. 
 * 
 * @file shamazon_robot.cpp
 * @author Maaruf Vazifdar
 * @author Maitreya Kulkarni
 * @author Pratik Bhujbal
 * @brief Implementation file for Shamazon Warehouse Robot.
 * @version 1.0
 * @date 11/26/2021
 * @copyright  Copyright (c) 2021
 * 
 */

#include<ros/ros.h>
#include<user_interface.hpp>
#include<navigation.hpp>
#include<elevator.hpp>
#include <ros/package.h>
#include "nav_msgs/MapMetaData.h"
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/SetModelState.h>
#include<iostream>


// void UserInterface::setDeliveryLocation(std::string delivery_location) {
//   std::cout << "Enter the goal you want the robot to go to:"<<std::endl;
//   std::cout << "0: Floor 1\n" << "0: Floor 1\n";
//   int G, final_goal;
//   std::cout << "Select Goal: (Please enter number 0 or 1): ";
//   std::cin >> G;
//   switch (G) {   
//     case 0:
//       Send_goal_to_action_server(8, 8, 3.14);
//         break;
//     case 1:
//       Send_goal_to_action_server(0,0,0);
//       break;
//     default:
//       cout << "Incorrect Goal !";
//       break;
// }

void UserInterface::setDeliveryFloor(int delivery_floor) {

}

int Elevator::getElevatorFloor() {

}

bool Elevator::moveElevator(int delivery_floor) {

}

// void Navigation::PoseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
//   double robot_position[] = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
//   double robot_quat[] = {msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};

//   robot_positionX = robot_position[0];
//   robot_positionY = robot_position[1];
// }

// void Navigation::sendGoalsToActionServer(float goal_x, float goal_y, float goal_theta) {   
//   //tell the action client that we want to spin a thread by default
//   MoveBaseClient ac("move_base", true);

//   //wait for the action server to come up
//   while(!ac.waitForServer(ros::Duration(5.0))){
//       ROS_INFO("Waiting for the move_base action server to come up");
//   }

//   move_base_msgs::MoveBaseGoal goal;

//   goal.target_pose.header.frame_id = "odom";
//   goal.target_pose.header.stamp = ros::Time::now();
//   goal.target_pose.pose.position.x = goal_x;
//   goal.target_pose.pose.position.y = goal_y;

//   euler_to_quaternion(0, 0, goal_theta);
//   goal.target_pose.pose.orientation.x = qx;
//   goal.target_pose.pose.orientation.y = qy;
//   goal.target_pose.pose.orientation.z = qz;
//   goal.target_pose.pose.orientation.w = qw;

//   // ROS_INFO("Sending goal");
//   ac.sendGoal(goal);

//   ac.waitForResult();

//   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//       ROS_INFO_STREAM("Goal Reached !\n");
//   else
//       ROS_ERROR_STREAM("Failed to reach goal !!!\n");
// }

bool Navigation::activateConveyor() {
  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  ros::Rate loop_rate(100);

  geometry_msgs::Point target_pose;
  geometry_msgs::Pose package_pose;
  gazebo_msgs::ModelState package_modelstate;

  for (double i = 0; i < 2000; i++) {
    target_pose.x = 2 + i / 1000;
    target_pose.y = 8;
    target_pose.z = 0.51;
    package_pose.position = target_pose;

    package_modelstate.model_name = (std::string) "package";
    package_modelstate.pose = package_pose;

    gazebo_msgs::SetModelState srv;
    srv.request.model_state = package_modelstate;
  }
  return true;
}

bool Navigation::activateRobotConveyor() {}

// void Navigation::updateGlobalMap(int floor_map,const nav_msgs::OccupancyGrid::ConstPtr &msg) {
//   if (floor_map==1){
//   map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
//   map_pub.publish(msg);}}
// }

int main(){
  Navigation N1;
  std::cout << "Status" << N1.activateConveyor();
  return 0;
}
