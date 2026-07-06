cmake_minimum_required(VERSION 3.5)
project(imu_mpu6050)

add_executable(imu_mpu6050 ./src/main.c)
target_link_libraries(imu_mpu6050 ufr)
install(TARGETS imu_mpu6050 RUNTIME DESTINATION bin)