cmake_minimum_required(VERSION 3.0.2)
project(hutb_pilotnet_e2e)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
