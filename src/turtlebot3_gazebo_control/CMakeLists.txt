cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_gazebo_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  gazebo_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)