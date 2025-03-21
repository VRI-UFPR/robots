# CPU Central

![alt text](../docs/cpu_central_graph.png)

## Requirements
```
pip3 install ultralytics
```

## Building and Execution
```
cd robot-home/cpu_central
colcon build
source install/setup.bash
ros2 launch object_recognition all.py
```


## Possible Problems

- Could not find a package configuration file provided by "ament_cmake" 
```
CMake Error at CMakeLists.txt:19 (find_package):
  By not providing "Findament_cmake.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "ament_cmake", but CMake did not find one.

  Could not find a package configuration file provided by "ament_cmake" with
  any of the following names:

    ament_cmakeConfig.cmake
    ament_cmake-config.cmake
```

Solution: source /opt/ros/humble/setup.bash

- ModuleNotFoundError: No module named 'ultralytics'
```
[INFO] [yolo_subscriber.py-3]: process started with pid [33315]
[yolo_node.py-2] Traceback (most recent call last):
[yolo_node.py-2]   File "/home/fgbombardelli/workspace/robot-home/cpu_central/install/object_recognition/lib/object_recognition/yolo_node.py", line 7, in <module>
[yolo_node.py-2]     from ultralytics import YOLO
[yolo_node.py-2] ModuleNotFoundError: No module named 'ultralytics'
```

Solution: pip3 install ultralytics

- Could not initialize NNPACK!
```
[INFO] [yolo_node.py-2]: process started with pid [33406]
[INFO] [yolo_subscriber.py-3]: process started with pid [33408]
100%|██████████| 6.23M/6.23M [00:00<00:00, 84.9MB/s]
[yolo_node.py-2] [W NNPACK.cpp:64] Could not initialize NNPACK! Reason: Unsupported hardware.

```
Possible solution not tested: recompile manually the nnpack with USE_NNPACK=0 
[Site de Referencia](https://discuss.pytorch.org/t/bug-w-nnpack-cpp-80-could-not-initialize-nnpack-reason-unsupported-hardware/107518)