cmake_minimum_required(VERSION 3.5)
project(simple_follower)

# Encontrar pacotes necessários
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(object_recognition REQUIRED)
find_package(std_msgs REQUIRED)

# Instalar scripts Python
install(
  PROGRAMS
  scripts/simple_follower.py  # Substitua pelo nome correto do seu script
  DESTINATION lib/${PROJECT_NAME}
)

# Ament packaging macros
ament_package()
