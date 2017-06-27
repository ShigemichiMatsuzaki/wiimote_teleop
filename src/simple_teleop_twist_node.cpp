#include <ros/ros.h>
// Standard
#include<iostream>

#include "include/robot_driver.h"

int main(int argc, char** argv){
  // Initialize ROS
  ros::init(argc, argv, "simple_teleop_twist");

  // Generate an instance of RobotDriver
  RobotDriver robotDriver;

  // Spin
  ros::spin();

  return 0;
}
