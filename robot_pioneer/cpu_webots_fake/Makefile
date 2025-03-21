SHELL := /bin/bash

# build and execution
install: ./src/my_package/my_package/my_robot_driver.py
	colcon build

run: install
	source /opt/ros/humble/setup.bash; source install/local_setup.bash; ros2 launch my_package robot_launch.py


# Control of robot
go:
	source /opt/ros/humble/setup.bash; ros2 topic pub -1 /cmd_vel  geometry_msgs/Twist  "linear: { x: 0.1 }"

back:
	source /opt/ros/humble/setup.bash; ros2 topic pub -1 /cmd_vel  geometry_msgs/Twist  "linear: { x: -0.1 }"

stop:
	source /opt/ros/humble/setup.bash; ros2 topic pub -1 /cmd_vel geometry_msgs/Twist  "linear: { x: 0.0 }"