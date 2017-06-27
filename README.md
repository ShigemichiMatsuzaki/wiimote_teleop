# About
Convert Joy message subscribed by wiimote to Twist message
Why did I make this kind of package?
Because teleop_twist_joy package was useless.

# How to launch
Launch the simulator of Turtlebot
```
 $ rosrun simple_teleop_twist simple_teleop_twist_node
```

# Messages
## Subscribed
/joy
Topic from a controller

## Published
/cmd_vel
Topic for mercury_converter, and eventually for mercury itself.

## Todo