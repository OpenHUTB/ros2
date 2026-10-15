cmake_minimum_required(VERSION 3.0.2)
project(yolo_detect)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
