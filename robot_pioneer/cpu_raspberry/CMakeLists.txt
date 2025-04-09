cmake_minimum_required(VERSION 3.22)
project(pioneer-raspberry)

include(../topicos.cmake)

add_subdirectory(src/app_base_pioneer)
# add_subdirectory(src/app_data)
add_subdirectory(src/app_lidar_rplidar)
# add_subdirectory(src/lib_slam_lama)
# add_subdirectory(src/app_slam_lama)