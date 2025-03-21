SHELL := /bin/bash

# build and execution
build: src/object_recognition/scripts/webcam_publisher.py src/object_recognition/scripts/yolo_node.py
	colcon build

run: build
	source /opt/ros/humble/setup.bash; source install/local_setup.bash; ros2 launch object_recognition all.py


