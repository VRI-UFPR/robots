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

## Possible Errors

- No module named 'webots_ros2_driver'
```
source /opt/ros/humble/setup.bash; source install/local_setup.bash; ros2 launch my_package robot_launch.py
[INFO] [launch]: All log files can be found below /home/fgbombardelli/.ros/log/2024-02-19-14-23-06-398164-pc23-12105
[INFO] [launch]: Default logging verbosity is set to INFO
[ERROR] [launch]: Caught exception in launch (see debug for traceback): Caught exception when trying to load file of format [py]: No module named 'webots_ros2_driver'
```

Solution: sudo apt install ros-humble-webots-ros2

- AttributeError: 'MyRobotDriver' object has no attribute '_MyRobotDriver__node'. Did you mean: '_MyRobotDriver__robot'?
```
[webots_controller_pioneer-2] Traceback (most recent call last):
[webots_controller_pioneer-2]   File "/home/fgbombardelli/workspace/robot-home/webots_cpu_rasp/install/my_package/lib/python3.10/site-packages/my_package/my_robot_driver.py", line 43, in step
[webots_controller_pioneer-2]     rclpy.spin_once(self.__node, timeout_sec=0)
[webots_controller_pioneer-2] AttributeError: 'MyRobotDriver' object has no attribute '_MyRobotDriver__node'. Did you mean: '_MyRobotDriver__robot'?
```

There is problem in the my_robot_driver.py. Probable, there is device with different name in my_robot_driver.py with the file world.wbt