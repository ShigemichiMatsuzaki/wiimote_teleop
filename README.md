# About
Convert Joy message subscribed by wiimote to Twist message

Why did I make this kind of package?

Because teleop_twist_joy package was useless.

# How to launch
1. Connect wiimote to your PC.
See [here](https://aisl-serv6.aisl.ics.tut.ac.jp:20443/ShigemichiMatsuzaki/research_notes/blob/master/notes/wiimote.md)

1. Run **simple_teleop_twist**
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
