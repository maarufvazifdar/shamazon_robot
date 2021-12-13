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
#include<ros/package.h>
#include<nav_msgs/MapMetaData.h>
#include<tf/tf.h>
#include<gazebo_msgs/SetModelState.h>
#include<iostream>
#include<std_msgs/Float64.h>


void UserInterface::setDeliveryLocation(std::string delivery_location) {
  // ros::ROS_INFO_STREAM("Location 1 selected!");
  std::cout << "Location 1 selected!";
  delivery_location = "Location 1";
}

void UserInterface::setDeliveryFloor(int delivery_floor) {
  // ros::ROS_INFO_STREAM("Floor " << delivery_floor <<" selected!");
  std::cout << "Floor " << delivery_floor << " selected!";
  delivery_floor = delivery_floor;
}

int Elevator::getElevatorFloor() {

}

bool Elevator::moveElevator() {
  Navigation N2;
  ros::ServiceClient client = N2.nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  ros::Rate loop_rate(100);

  // geometry_msgs::Point target_pose;
  geometry_msgs::Pose robot_pose;
  gazebo_msgs::ModelState robot_modelstate;
  geometry_msgs::Pose elevator_pose;
  gazebo_msgs::ModelState elevator_modelstate;
  geometry_msgs::Pose package_pose;
  gazebo_msgs::ModelState package_modelstate;

  gazebo_msgs::SetModelState srv;
  gazebo_msgs::SetModelState srv2;
  gazebo_msgs::SetModelState srv3;
  

  for (double i = 0; i < 4000; i++) {
    robot_pose.position.x = 0;
    robot_pose.position.y = 0;
    robot_pose.position.z = 0.2 + i/1000;
    robot_pose.orientation.x = 0;
    robot_pose.orientation.y = 0;
    robot_pose.orientation.z = 1;
    robot_pose.orientation.w = 0;
    elevator_pose.position.z = i / 1000;
    package_pose.position.z = 0.5 + i/1000;

    robot_modelstate.model_name = (std::string) "shamazon_robot";
    robot_modelstate.pose = robot_pose;

    elevator_modelstate.model_name = (std::string) "elevator";
    elevator_modelstate.pose = elevator_pose;

    package_modelstate.model_name = (std::string) "package";
    package_modelstate.pose = package_pose;
        
    srv.request.model_state = robot_modelstate;
    srv2.request.model_state = elevator_modelstate;
    srv3.request.model_state = package_modelstate;
  
    if (client.call(srv) && client.call(srv2) && client.call(srv3)) {
      ROS_DEBUG_STREAM("Moving");
    } else {
        ROS_ERROR("Failed to magic move elevator! Error msg:%s",srv.response.status_message.c_str());
    }
  }
  geometry_msgs::Pose elevator_base_pose;
  gazebo_msgs::ModelState elevator_base_modelstate;
  gazebo_msgs::SetModelState srv4;
  elevator_base_pose.position.x = 0;
  elevator_base_pose.position.y = 0;
  elevator_base_pose.position.z = 0;
  elevator_base_modelstate.model_name = (std::string) "elevator_base";
  elevator_base_modelstate.pose = elevator_base_pose;
  srv4.request.model_state = elevator_base_modelstate;
  client.call(srv4);

  for (double i = 0; i < 4000; i++) {
    robot_pose.position.x = -i/2000;
    robot_pose.position.y = 0;
    robot_pose.position.z = 4.15;
    robot_pose.orientation.x = 0;
    robot_pose.orientation.y = 0;
    robot_pose.orientation.z = 1;
    robot_pose.orientation.w = 0;
    package_pose.position.x = -i/2000;
    package_pose.position.z = 4.5;

    robot_modelstate.model_name = (std::string) "shamazon_robot";
    robot_modelstate.pose = robot_pose;
    
    package_modelstate.model_name = (std::string) "package";
    package_modelstate.pose = package_pose;
    
    srv.request.model_state = robot_modelstate;
    srv3.request.model_state = package_modelstate;
  
    if (client.call(srv) && client.call(srv3)) {
      ROS_DEBUG_STREAM("Moving");
    } else {
        ROS_ERROR("Failed to magic move elevator! Error msg:%s",srv.response.status_message.c_str());
    }
  }
  return true;
}

void Navigation::PoseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  current_position_x = msg->pose.pose.position.x;
  current_position_y = msg->pose.pose.position.y;
  
  double current_quat[] = {msg->pose.pose.orientation.x, 
                           msg->pose.pose.orientation.y, 
                           msg->pose.pose.orientation.z, 
                           msg->pose.pose.orientation.w};

}

void Navigation::sendGoalsToActionServer(float goal_x, float goal_y, float goal_theta) {
  //tell the action client that we want to spin a thread by default
  _move_base_client ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
      // ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = goal_x;
  goal.target_pose.pose.position.y = goal_y;

  tf::Quaternion q;
  q.setRPY(0, 0, goal_theta);
  goal.target_pose.pose.orientation.x = q[0];
  goal.target_pose.pose.orientation.y = q[1];
  goal.target_pose.pose.orientation.z = q[2];
  goal.target_pose.pose.orientation.w = q[3];

  // ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      // ros::ROS_INFO_STREAM("Goal Reached !\n");
      std::cout << "Goal Reached !\n";
  else
    // ROS_ERROR_STREAM("Failed to reach goal !!!\n");
    std::cout << "Failed to reach goal\n";
}

bool Navigation::activateConveyor() {
  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  ros::Rate loop_rate(100);
  geometry_msgs::Point target_pose;
  geometry_msgs::Pose package_pose;
  gazebo_msgs::ModelState package_modelstate;

  for (double i = 0; i < 6000; i++) {
    target_pose.x = 1 + i / 2000;
    target_pose.y = 8;
    target_pose.z = 0.505;
    package_pose.position = target_pose;

    package_modelstate.model_name = (std::string) "package";
    package_modelstate.pose = package_pose;

    gazebo_msgs::SetModelState srv;
    srv.request.model_state = package_modelstate;
    if (client.call(srv)) {
        ROS_DEBUG_STREAM("Moving conveyor");
      } else {
          ROS_ERROR("Failed to magic move conveyor! Error msg:%s",srv.response.status_message.c_str());
      }
  }
  return true;
}

bool Navigation::activateRobotConveyor() {
  ros::Publisher rob_pub = nh.advertise<std_msgs::Float64>("/conveyor2_joint_controller/command", 10);
  std_msgs::Float64 msg;
  
  while (rob_pub.getNumSubscribers() < 2) {
    msg.data = 0.7;
    rob_pub.publish(msg);
  }
  return true;
}

// void Navigation::updateGlobalMap(int floor_map,const nav_msgs::OccupancyGrid::ConstPtr &msg) {
//   if (floor_map==1){
//   map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
//   map_pub.publish(msg);}}
// }

int main(int argc, char** argv) {
  ros::init(argc, argv,"shamazon_robot");
  
  Navigation N1;
  UserInterface UI1;
  Elevator E1;
  // ros::ROS_INFO_STREAM("Select delivery floor :\n");
  // ros::ROS_INFO_STREAM("1: Floor 1\n" << "2: Floor 2\n");
  // std::cout<<"Select delivery floor :\n";
  // std::cout<<"1: Floor 1\n" << "2: Floor 2\n";
  // int floor_no;
  // std::cin >> floor_no;
  // UI1.setDeliveryFloor(floor_no);
  // // ros::ROS_INFO_STREAM("Select delivery Goal: (We only have 1 goal per floor for now): ");
  // std::cout<<"\nSelect delivery Goal: (We only have 1 goal per floor for now): ";
  // std::string delivery_loc;
  // std::cin >> delivery_loc;
  // UI1.setDeliveryLocation(delivery_loc);

  // Send robot to pickup package  
  N1.sendGoalsToActionServer(4, 8, 1.57);
  N1.activateConveyor();

  N1.sendGoalsToActionServer(4.5,8.6,0);
  N1.sendGoalsToActionServer(0,0,3.14); // elevator goal
  E1.moveElevator();
  N1.sendGoalsToActionServer(-5,0,3.14);
  N1.activateRobotConveyor();

  // if (floor_no == 1) {
  //   N1.sendGoalsToActionServer(5,0,0);   //floor 1 delivery goal
  //   N1.activateRobotConveyor();
  // } else if (floor_no == 2) {
  //   N1.sendGoalsToActionServer(0,0,3.14); // elevator goal
  //   N1.activateRobotConveyor();
    // N1.moveElevator();
    // N1.sendGoalsToActionServer();  //floor 2 delivery goal
  // }

  return 0;
}
