# Simulator for Pioneer

These files are based on this [a tutorial for ROS+Webots](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html)

## Requirements
```
sudo apt install webots
sudo apt install ros-humble-webots-ros2
```

## Building and Running
```
colcon build
source install/local_setup.bash
ros2 launch my_package robot_launch.py
```

## Send message
```
ros2 topic pub /cmd_vel geometry_msgs/Twist  "linear: { x: 0.1 }"
```
