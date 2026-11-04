cmake_minimum_required(VERSION 3.0.2)
project(turtle_heart)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  turtlesim
)

catkin_package()

catkin_install_python(PROGRAMS
  main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
