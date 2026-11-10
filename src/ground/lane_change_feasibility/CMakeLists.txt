cmake_minimum_required(VERSION 3.0.2)
project(lane_change_feasibility)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf2_ros
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/lane_change_core.py
  scripts/lane_change_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
