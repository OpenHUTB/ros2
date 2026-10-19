cmake_minimum_required(VERSION 3.0.2)
project(ros_topic_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()
