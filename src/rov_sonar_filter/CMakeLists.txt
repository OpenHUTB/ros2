cmake_minimum_required(VERSION 3.8)
project(rov_sonar_filter)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  scripts/filter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()