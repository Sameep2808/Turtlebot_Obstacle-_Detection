///============================================================================
/// @file        : sub.cpp
/// @version     : 1.0
/// @author      : Sameep Pote
/// @copyright   : MIT License
/// @brief       : sub.cpp This file is used to receive the custom string
///============================================================================

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

ros::Publisher pub;
void chatterCallback(const sensor_msgs::LaserScan::ConstPtr &data) {
  geometry_msgs::Twist pos;

  for (int i = 0; i < 60; i++) {
    if (data->ranges[i] <= 0.4) {
      ROS_INFO("Object Detected left");
      pos.linear.x = -0.20;
      pos.angular.z = -2;
    } else if (data->ranges[i + 300] <= 0.4) {
      ROS_INFO("Object Detected right");
      pos.linear.x = -0.20;
      pos.angular.z = 2;
    } else {
      ROS_INFO("Move");
      pos.linear.x = 0.2;
      pos.angular.z = 0.0;
    }
  }
  pub.publish(pos);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "move");

  ros::NodeHandle n;
  pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Subscriber sub = n.subscribe("/scan", 1, chatterCallback);
  ros::Rate loop_rate(10);
  ros::spin();

  return 0;
}
