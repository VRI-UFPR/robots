
# add_executable(loc2d_ros loc2d_ros.cpp)
# target_link_libraries(loc2d_ros iris_lama::iris_lama)

# add_executable(slam2d_ros slam2d_ros.cpp offline_replay.cpp)
# target_link_libraries(slam2d_ros iris_lama::iris_lama)

# add_executable(pf_slam2d_ros pf_slam2d_ros.cpp offline_replay.cpp)
# target_link_libraries(pf_slam2d_ros iris_lama::iris_lama)

# add_executable(lidar_odometry2d_ros lidar_odometry2d_ros.cpp offline_replay.cpp)
# target_link_libraries(lidar_odometry2d_ros ${catkin_LIBRARIES} iris_lama::iris_lama)

# add_executable(ros2sdm ros2sdm.cpp)
# target_link_libraries(ros2sdm ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} iris_lama::iris_lama)


# install(TARGETS loc2d_ros slam2d_ros pf_slam2d_ros graph_slam2d_ros ros2sdm lidar_odometry2d_ros
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

