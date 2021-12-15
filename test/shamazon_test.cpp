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
 * @file shamazon_test.cpp
 * @author Pratik Bhujbal
 * @author Maaruf Vazifdar
 * @author Maitreya Kulkarni
 * @brief test file.
 * @version 1.0
 * @date 11/26/2021
 * @copyright  Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/service_client.h>
#include <gazebo_msgs/SetModelState.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <navigation.hpp>
std::shared_ptr<ros::NodeHandle> nh;

// Gazebo set model state service test
TEST(ServiceCheck, service_check) {
  ros::service::waitForService("/gazebo/set_model_state");
  ros::ServiceClient client = nh->serviceClient
  <gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);
}

// Elevator Service Test
TEST(ServiceCheckRun, elevator_service_test) {
    ros::ServiceClient client = nh->serviceClient
    <gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState srv;
    geometry_msgs::Pose elevator_pose;
    gazebo_msgs::ModelState elevator_modelstate;
    elevator_pose.position.z = 0.1;
    elevator_modelstate.model_name = (std::string) "elevator";
    elevator_modelstate.pose = elevator_pose;
    srv.request.model_state = elevator_modelstate;
    client.call(srv);
    bool check = srv.response.success;
    EXPECT_TRUE(check);
  }

// MoveBase action server
TEST(MoveBaseActionServerTest, move_base_actionserver) {
  typedef actionlib::SimpleActionClient
  <move_base_msgs::MoveBaseAction> MoveBaseClient;
  MoveBaseClient ac("move_base", true);
  bool movebase_exists(ac.waitForServer(ros::Duration(0.5)));
  EXPECT_FALSE(movebase_exists);
}

// MoveBase Goal Test
TEST(MoveBaseActionServerTest, test_goal) {
  typedef actionlib::SimpleActionClient
  <move_base_msgs::MoveBaseAction> MoveBaseClient;
  MoveBaseClient ac("move_base", true);
  while (!ac.waitForServer(ros::Duration(0.5))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 2.1;
  goal.target_pose.pose.position.y = -8;
  goal.target_pose.pose.orientation.w = 1.0;
  ac.sendGoal(goal);
  ac.waitForResult();
  EXPECT_EQ(ac.getState(), actionlib::SimpleClientGoalState::SUCCEEDED);
}

//  Package Feed Conveyor Service Test
TEST(ServiceCheckConveyor, conveyor_service_test) {
    ros::ServiceClient client = nh->serviceClient
    <gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState srv;
    geometry_msgs::Pose conveyor_pose;
    gazebo_msgs::ModelState conveyor_modelstate;
    conveyor_pose.position.x = 0.1;
    conveyor_modelstate.model_name = (std::string) "conveyor";
    conveyor_modelstate.pose = conveyor_pose;
    srv.request.model_state = conveyor_modelstate;
    client.call(srv);
    bool check = srv.response.success;
    EXPECT_TRUE(check);
  }


int main(int argc, char **argv) {
  ros::init(argc, argv, "shamazon_test_node");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
