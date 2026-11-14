cmake_minimum_required(VERSION 3.0.2)
project(delivery_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/octree_delivery_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

