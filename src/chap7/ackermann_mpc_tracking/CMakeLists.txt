cmake_minimum_required(VERSION 3.8)
project(ackermann_mpc_tracking)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

install(PROGRAMS
  scripts/ros_node.py
  scripts/main.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/mpc.py
  scripts/simulate.py
  scripts/stanley_baseline.py
  scripts/simple_plot.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md requirements.txt
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
