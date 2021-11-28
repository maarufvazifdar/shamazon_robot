#include<ros/ros.h>
#include<nav_msgs/Odometry.h> 
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#pragma once


class Navigation {
 public:
  ros::NodeHandle nh;
  
  Navigation() {
    _goal_status = true;
    _pickup_goal = [1, 0, 1.5707];
    _delivery_goal = [15, 5, 3.1415];
  }

  ~Navigation() {}

  void PoseCallback(const nav_msgs::Odometry::ConstPtr &msg);

  void sendGoalsToActionServer(float goal_x, float goal_y, float goal_theta);

  bool activateConveyor();

  bool activateRobotConveyor();

  void updateGlobalMap(int floor_map);

 private:
  bool _goal_status;
  double _pickup_goal[];
  double _delivery_goal[];
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> _move_base_client;
  move_base_msgs::MoveBaseGoal _goal;
  nav_msgs::Odometry _robot_pose;
};
