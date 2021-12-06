/**
 * BSD-3 Clause
 *
 * Copyright (c) 2021 Maaruf Vazifdar, Maitreya Kulkarni, Pratik Bhujnbal
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
 * @file navigation.hpp
 * @author Maaruf Vazifdar
 * @author Maitreya Kulkarni
 * @author Pratik Bhujnbal
 * @brief Class to hold robot's navigation related attributes and members.
 * @version 1.0
 * @date 11/26/2021
 * @copyright  Copyright (c) 2021
 * 
 */

#include<ros/ros.h>
#include<nav_msgs/Odometry.h> 
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>
#include <tf/tf.h>


#pragma once


/**
 * @brief Class to hold robot's navigation related attributes and members.
 */
class Navigation {
 public:
  ros::NodeHandle nh;
  
  /** 
   * @brief  Constrctor of Class Navigation to initialize attributes.
   */
  Navigation() {
    _goal_status = true;
    // _pickup_goal = [1, 0, 1.5707];
    // _delivery_goal = [15, 5, 3.1415];
  }

  /**
   * @brief Destructor of class Navigation.
   */
  ~Navigation() {}

  /**
   * @brief Callback for current robot's pose.
   * @param const nav_msgs::Odometry::ConstPtr &msg - Robot odometry data.
   * @return void
   */
  void PoseCallback(const nav_msgs::Odometry::ConstPtr &msg);

  /**
   * @brief Send desired goal to move_base action server.
   * @param goal_x float - X coordinate of desired goal (in m).
   * @param goal_y float - Y coordinate of desired goal (in m).
   * @param goal_theta float - Orientation of desired goal (in rad).
   * @return void
   */
  void sendGoalsToActionServer(float goal_x, float goal_y, float goal_theta);

  /**
   * @brief Command package feed conveyor to drop package on the robot. 
   * @param void
   * @return true - package feed conveyor status. 
   */
  bool activateConveyor();

  /**
   * @brief Command robot's conveyor to drop package on the delivery location. 
   * @param void
   * @return true - robot's conveyor status. 
   */
  bool activateRobotConveyor();

  /**
   * @brief Update global map in the map server.
   * @param floor_map int - Global map of specific floor.
   * @return void
   */
  void updateGlobalMap(int floor_map);

 private:
  /**
   * Class local variables.
   */
  bool _goal_status;
  // double _pickup_goal[];
  // double _delivery_goal[];
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> _move_base_client;
  move_base_msgs::MoveBaseGoal _goal;
  nav_msgs::Odometry _robot_pose;
};
