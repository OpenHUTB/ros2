cmake_minimum_required(VERSION 3.0.2)
project(hutb_grid_slam_nav)

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
