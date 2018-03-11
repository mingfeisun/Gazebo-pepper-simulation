# Gazebo-pepper-simulation

### Installation


### How to use it

*Start simulation*
```bash
roslaunch pepper_gazebo_plugin pepper_gazebo_plugin_Y20.launch
```

*Control*
```bash
# example
rostopic pub /pepper_dcm/HeadYaw_position_controller/command std_msgs/Float64 "1"

# 

```
