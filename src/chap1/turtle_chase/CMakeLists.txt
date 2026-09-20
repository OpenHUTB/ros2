cmake_minimum_required(VERSION 3.0.2)
project(turtle_chase)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  geometry_msgs
  turtlesim
)

catkin_package()

catkin_install_python(PROGRAMS
  main.py
  scripts/runner.py
  scripts/chaser.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
