cmake_minimum_required(VERSION 3.0.2)
project(ros_topic_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs nav_msgs geometry_msgs tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/trajectory_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
