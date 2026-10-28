cmake_minimum_required(VERSION 3.0.2)
project(fixed_point_stop)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslaunch
)

catkin_package(CATKIN_DEPENDS rospy sensor_msgs nav_msgs geometry_msgs)

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()

install(PROGRAMS main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
