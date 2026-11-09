cmake_minimum_required(VERSION 3.8)
project(ros2_lifecycle_node)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

install(PROGRAMS
  scripts/ros_node.py
  scripts/main.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/lifecycle.py
  scripts/simulate.py
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
