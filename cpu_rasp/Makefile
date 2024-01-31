SHELL := /bin/bash

all:
	colcon build --symlink-install --packages-select rosaria_msgs
	source install/local_setup.bash; colcon build --symlink-install --packages-select rosaria

clean:
	rm -rf build
	rm -rf install
	rm -rf log