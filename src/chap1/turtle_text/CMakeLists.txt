cmake_minimum_required(VERSION 3.0.2)
project(turtle_text)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  turtlesim
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/draw_text.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)