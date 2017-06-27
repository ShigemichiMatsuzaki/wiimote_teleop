/*
 * RobotDriver.cpp
 *
 *  Created on: 2017/06/27
 *      Author: ShigemichiMatsuzaki
 */

// My own header files
#include "include/robot_driver.h"
// ROS header files
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
// Standard 
#include<iostream>

// Indices for array buttons
#define ONE   0
#define TWO   1
#define A     2
#define B     3
#define PLUS  4
#define MINUS 5
#define LEFT  6
#define RIGHT 7
#define UP    8
#define DOWN  9
#define HOME  10

/**
 * Constructor
 *  - Generate an instance of publisher and subscriber
 *  - Initialize the twist message and publish it
 */
RobotDriver::RobotDriver(){
    twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    msg_sub = nh.subscribe("/joy", 1, &RobotDriver::callback, this);

    geometry_msgs::Twist twist = setMoveVector(); // Initialize the vector

    twist_pub.publish(twist);

    // Initialize the speed of rotation
    ros::param::set("rotateSpeed", 1);
    // Initialize the speed of linear movement
    ros::param::set("linearSpeed", 0.5);
    
    // 
    ros::param::set("isPressed", false);
}

/**
 * Destructor
 *  - Does nothing. IDIOT
 */
RobotDriver::~RobotDriver(){}

/**
 * Function that is given to subscriber
 * This gets point cloud data, analyzes it and determines the values of Twist message to control the robot
 *
 * linear: + is forward
 * angular: + is counterclockwise
 */
void
RobotDriver::callback(const sensor_msgs::Joy& msg){
    geometry_msgs::Twist twist;

    twist = setMoveVector(msg);

    twist_pub.publish(twist);
}

/**
 * Function that set the values of Twist message to control the robot
 */
geometry_msgs::Twist
RobotDriver::setMoveVector(const sensor_msgs::Joy& msg){
    geometry_msgs::Twist twist = setMoveVector();
    
    float rotateSpeed;
    float linearSpeed;
    bool  isPressed;
    ros::param::get("rotateSpeed", rotateSpeed);
    ros::param::get("linearSpeed", linearSpeed);
    ros::param::get("isPressed",   isPressed);
    
    if(msg.buttons[UP]) {
      twist.linear.x = linearSpeed;
    } else if(msg.buttons[DOWN]) {
      twist.linear.x = -linearSpeed; 
    } else if(msg.buttons[LEFT]) {
      twist.angular.z = rotateSpeed;
    } else if(msg.buttons[RIGHT]) {
      twist.angular.z = -rotateSpeed;
    } else if(msg.buttons[B]) { // Maybe not necessary
      twist.linear.x = 0;
      twist.angular.z = 0;
    } 
      
    if(msg.buttons[PLUS] && !isPressed) {
      if(rotateSpeed <= 1 && linearSpeed <= 0.5) {
	rotateSpeed *= 1.1;
	linearSpeed *= 1.1;
	ros::param::set("rotateSpeed", rotateSpeed);
	ros::param::set("linearSpeed", linearSpeed);
      }
      ROS_INFO("linearSpeed : %f", linearSpeed);
      ROS_INFO("rotateSpeed : %f", rotateSpeed);
      ros::param::set("isPressed", true);
      
    } else if(msg.buttons[MINUS] && !isPressed) {
      rotateSpeed *= 0.9;
      linearSpeed *= 0.9;
      ros::param::set("rotateSpeed", rotateSpeed);
      ros::param::set("linearSpeed", linearSpeed);
      ROS_INFO("linearSpeed : %f", linearSpeed);
      ROS_INFO("rotateSpeed : %f", rotateSpeed);
      ros::param::set("isPressed", true);
    } else if(msg.buttons[PLUS] && msg.buttons[MINUS]) {
      ros::param::set("isPressed", false);
    }

    return twist;
}

geometry_msgs::Twist
RobotDriver::setMoveVector(){
  geometry_msgs::Twist twist;

  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
  
  return twist;
}

