/*
 * RobotDriver.h
 *
 *  Created on: 2017/04/19
 *      Author: Shigemichi Matsuzaki
 */

#ifndef ROS_EXERCISE_SRC_ROBOT_DRIVER_H_
#define ROS_EXERCISE_SRC_ROBOT_DRIVER_H_

// ROS header files
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class RobotDriver{
  public:
    // Constructor
    RobotDriver();
    // Destructor
    ~RobotDriver();
    // Main function that is called when a message is subscribed
    void callback(const sensor_msgs::Joy& msg);

  private:
    // Set the vector to publish to control the robot based on the message subscribed
    geometry_msgs::Twist setMoveVector(const sensor_msgs::Joy& msg);
    geometry_msgs::Twist setMoveVector();
    
    ros::Publisher twist_pub; // Publisher for controling the robot
    ros::Subscriber msg_sub; // Subscriber for messages coming from object detecter.
    ros::NodeHandle nh; // NodeHandle to generate an instance of publisher and subscriber
};
#endif /* ROS_EXERCISE_SRC_ROBOT_DRIVER_H_ */
