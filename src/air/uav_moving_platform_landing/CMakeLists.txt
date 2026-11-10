cmake_minimum_required(VERSION 3.0.2)
project(uav_moving_platform_landing)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/moving_landing_node.py
  scripts/moving_landing_core.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config tools
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
