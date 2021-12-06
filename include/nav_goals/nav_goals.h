#ifndef NAV_GOALS_H
#define NAV_GOALS_H

#include<nav_msgs/Odometry.h> 
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include<ros/ros.h>           
#include<iostream>
#include<math.h>
#include <vector>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

class Send_goal
{   public:
        vector<vector<double>> table_1 = {{3.4, 3.4, -2}, {5, 1.2, -2.5}, {5.6, 4, -1}, {5, 4.8, 1.8}};
        vector<vector<double>> table_2 = {{3.4, 7, -2}, {5, 5.4, -2.5}, {5.6, 7, -1}};
        vector<vector<double>> table_3 = {{3.4, 9, -2}, {5, 9.5, 1.8}, {5.6, 9, 0}};
        vector<vector<double>> table_4 = {{6.4, 3.4, -2}, {7.8, 1.2, -2.5}, {8.6, 4, -1}, {7.8, 4.8, 1.8}};
        vector<vector<double>> table_5 = {{6.4, 7, -2}, {7.8, 5.4, -2.5}, {8.6, 7, -1}};
        vector<vector<double>> table_6 = {{6.4, 9, -2}, {7.8, 9.5, 1.8}, {8.6, 9, 0}};
        vector<vector<double>> room_1 = {{7.5, 0, 1.57}};
        vector<vector<double>> room_2 = {{12, 0, 1.8}};
        vector<vector<double>> room_3 = {{10, 14, -1.8}};

        double qx, qy, qz, qw, roll, pitch, yaw, robot_positionX, robot_positionY;
        int index;
        
        void quaternion_to_euler(double qx, double qy, double qz, double qw) {   
            double t0 = +2.0 * (qw * qx + qy * qz);
            double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
            double roll = atan2(t0, t1);

            double t2 = +2.0 * (qw * qy - qz * qx);
            double pitch = asin(t2);

            double t3 = +2.0 * (qw * qz + qx * qy);
            double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
            double yaw = atan2(t3, t4);
        }

        void euler_to_quaternion(double roll, double pitch, double yaw) {
            qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
            qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
            qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
            qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
        }

        int nearest_goal(vector<vector<double>> goals, double x_current, double y_current) {
            index = 0;
            double distance,  min_distance;
            vector<double> distance_to_goal;

            for (int i = 0; i < goals.size(); i++)
            {
                distance = sqrt(pow((goals[i][0] - x_current), 2) + pow((goals[i][1] - y_current), 2));
                distance_to_goal.push_back(distance);
            }

            min_distance = distance_to_goal[0];
            for (int j = 0; j < distance_to_goal.size(); j++)
            {   if (distance_to_goal[j] < min_distance)
                {   min_distance = distance_to_goal[j];
                    index = j;
                }
            }
            return index;
        }

        void PoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
        {   
            // Function to get the current robot pose
            double robot_position[] = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
            double robot_quat[] = {msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};

            robot_positionX = robot_position[0];
            robot_positionY = robot_position[1];
        }

        void select_goal()
        {
            cout << "Enter the goal you want the robot to go to:"<<endl;
            cout << "0: Home \n"
                 << "1: Table 1\n"
                 << "2: Table 2\n"
                 << "3: Table 3\n"
                 << "4: Table 4\n"
                 << "5: Table 5\n"
                 << "6: Table 6\n"
                 << "7: Room 1\n"
                 << "8: Room 2\n"
                 << "9: Room 3\n";
            int G, final_goal;
            cout << "Select Goal: (Please enter numbers between 0-9) ";
            cin >> G;
            Send_goal s2;

            switch (G)
            {   case 0:
                    Send_goal_to_action_server(0, 0, 0);
                    break;
                case 1:
                    final_goal = nearest_goal(table_1, robot_positionX, robot_positionY);
                    Send_goal_to_action_server(table_1[final_goal][0], table_1[final_goal][1], table_1[final_goal][2]);
                    break;
                case 2:
                    final_goal = nearest_goal(table_2, robot_positionX, robot_positionY);
                    Send_goal_to_action_server(table_2[final_goal][0], table_2[final_goal][1], table_2[final_goal][2]);
                    break;
                case 3:
                    final_goal = nearest_goal(table_3, robot_positionX, robot_positionY);
                    Send_goal_to_action_server(table_3[final_goal][0], table_3[final_goal][1], table_3[final_goal][2]);
                    break;
                case 4:
                    final_goal = nearest_goal(table_4, robot_positionX, robot_positionY);
                    Send_goal_to_action_server(table_4[final_goal][0], table_4[final_goal][1], table_4[final_goal][2]);
                    break;
                case 5:
                    final_goal = nearest_goal(table_5, robot_positionX, robot_positionY);
                    Send_goal_to_action_server(table_5[final_goal][0], table_5[final_goal][1], table_5[final_goal][2]);
                    break;
                case 6:
                    final_goal = nearest_goal(table_6, robot_positionX, robot_positionY);
                    Send_goal_to_action_server(table_6[final_goal][0], table_6[final_goal][1], table_6[final_goal][2]);
                    break;
                case 7:
                    final_goal = nearest_goal(room_1, robot_positionX, robot_positionY);
                    Send_goal_to_action_server(room_1[final_goal][0], room_1[final_goal][1], room_1[final_goal][2]);
                    break;
                case 8:
                    final_goal = nearest_goal(room_2, robot_positionX, robot_positionY);
                    Send_goal_to_action_server(room_2[final_goal][0], room_2[final_goal][1], room_2[final_goal][2]);
                    break;
                case 9:
                    final_goal = nearest_goal(room_3, robot_positionX, robot_positionY);
                    Send_goal_to_action_server(room_3[final_goal][0], room_3[final_goal][1], room_3[final_goal][2]);
                    break;
                default:
                    cout << "Incorrect Goal !";
                    break;
            }
        }

        void Send_goal_to_action_server(double goal_x, double goal_y, double goal_theta)
            {   
                //tell the action client that we want to spin a thread by default
                MoveBaseClient ac("move_base", true);

                //wait for the action server to come up
                while(!ac.waitForServer(ros::Duration(5.0))){
                    ROS_INFO("Waiting for the move_base action server to come up");
                }

                move_base_msgs::MoveBaseGoal goal;

                goal.target_pose.header.frame_id = "odom";
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position.x = goal_x;
                goal.target_pose.pose.position.y = goal_y;

                euler_to_quaternion(0, 0, goal_theta);
                goal.target_pose.pose.orientation.x = qx;
                goal.target_pose.pose.orientation.y = qy;
                goal.target_pose.pose.orientation.z = qz;
                goal.target_pose.pose.orientation.w = qw;

                // ROS_INFO("Sending goal");
                ac.sendGoal(goal);

                ac.waitForResult();

                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    ROS_INFO("Goal Reached !\n");
                else
                    ROS_INFO("Failed to reach goal !!!\n");
            }
};
#endif