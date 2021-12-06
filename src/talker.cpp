#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/SetModelState.h"
#include <sstream>

int main(int argc, char **argv)
{  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

 
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  ros::Rate loop_rate(100);

  geometry_msgs::Point target_pose;
  geometry_msgs::Pose elevator_pose;
  gazebo_msgs::ModelState elevator_modelstate;

  for (double i = 0; i < 4000; i++) {
    target_pose.z = i/1000;
    elevator_pose.position = target_pose;

    elevator_modelstate.model_name = (std::string) "elevator";
    elevator_modelstate.pose = elevator_pose;

    gazebo_msgs::SetModelState srv;
    srv.request.model_state = elevator_modelstate;

    if (client.call(srv)) {
      ROS_INFO_STREAM("Moving");
    } else {
        ROS_ERROR("Failed to magic move PR2! Error msg:%s",srv.response.status_message.c_str());
    }
  }
  // target_pose.z = 4;
  // elevator_pose.position = target_pose;

  // elevator_modelstate.model_name = (std::string) "elevator";
  // elevator_modelstate.pose = elevator_pose;

  // gazebo_msgs::SetModelState srv;
  // srv.request.model_state = elevator_modelstate;
  return 0;
}
 