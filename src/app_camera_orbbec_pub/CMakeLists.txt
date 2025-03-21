cmake_minimum_required(VERSION 3.5)
project(app_orbbec_pub)

find_package(OpenCV)
include_directories( /usr/include/openni2/ )

# Example 1: simple
add_executable(app_orbbec_pub ./src/main.cpp)
target_link_libraries(app_orbbec_pub ufr ${OpenCV_LIBS} OpenNI2)
# install(TARGETS app_orbbec_pub DESTINATION bin)
