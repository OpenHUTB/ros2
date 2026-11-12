cmake_minimum_required(VERSION 3.8)
project(rov_astar_planner)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  scripts/astar_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()