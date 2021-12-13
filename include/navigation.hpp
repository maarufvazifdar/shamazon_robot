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
#include <vector>
#include "nav_msgs/OccupancyGrid.h"

#pragma once

/**
 * @brief Class to hold robot's navigation related attributes and members.
 */
class Navigation {
 public:
  // ros::init(argc, argv,"shamazon_robot");
  ros::NodeHandle nh;
  std::vector<double> pickup_goal = {3.4, 3.4, -2};
  std::vector<std::vector<double>> delivery_goal= {{7.5, 0, 1.57},
                                                  {7.5, 0, 1.57}};

  /** 
   * @brief  Constrctor of Class Navigation to initialize attributes.
   */
  Navigation() {
    _goal_status = true;
    current_position_x;
    current_position_y;
    ros::Subscriber sub = nh.subscribe < nav_msgs::OccupancyGrid
      > ("/multimap_server/maps/level_2/localization/map", 1, &Navigation::mapCallback, this);
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
   * @param const nav_msgs::Odometry::ConstPtr &msg.
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
   * @param none.
   * @return void
   */
  void mapCallback(nav_msgs::OccupancyGrid msg);
  
  void updateGlobalMap();

 private:
  /**
   * Class local variables.
   */
  bool _goal_status;
  double current_position_x;
  double current_position_y;
  double current_quat[3];
  
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> _move_base_client;
  move_base_msgs::MoveBaseGoal _goal;
  nav_msgs::Odometry _robot_pose;
  nav_msgs::OccupancyGrid map;
};
