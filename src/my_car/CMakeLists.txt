cmake_minimum_required(VERSION 3.10)
project(my_car)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  urdf
  rviz
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy urdf rviz
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)