# About
Convert Joy message subscribed by wiimote to Twist message

## Docker

Coming soon...

## How to launch
1. Connect a wiimote to your PC.

1. Run **simple_teleop_twist**
```
 $ rosrun simple_teleop_twist simple_teleop_twist_node
```

## Messages
### Subscribed
/joy ([sensor_msgs/Joy](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Joy.html))

Topic from a controller

### Published
/cmd_vel ([sensor_msgs/Twist](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Twist.html))

Control command sent to a robot
