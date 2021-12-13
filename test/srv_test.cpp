#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/ServiceFile.h"
#include<navigation.hpp>
#include<elevator.hpp>

std::shared_ptr<ros::NodeHandle> nh;

TEST (TESTSuite, serviceFileCheck)
{
  Navigation N2;
  ros::ServiceClient client = N2.nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  bool exists(client.waitForExixtence(ros::Duration(1)));
  EXPECT_TRUE(exists);
}

TEST (TESTsuite, serviceTestOutput)
  {
    Navigation N2;
    ros::ServiceClient client = N2.nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState srv;
    geometry_msgs::Pose elevator_pose;
    gazebo_msgs::ModelState elevator_modelstate;
    elevator_pose.position.z = 0.1;
    elevator_modelstate.model_name = (std::string) "elevator";
    elevator_modelstate.pose = elevator_pose;
    
    srv.request.modelstate = elevator_modelstate;
    client.call(srv);
    bool check = srv.respond.modelstate;
    EXPECT_TRUE(check);
  }

// int main(int argc, char **argv) {
//   ros::init(argc, argv, "service_test_node");
//   nh.reset(new ros::NodeHandle);
//   testing::InitGoogleTest(&argc, argv);
//   return RUN_ALL_TESTS();
// }
