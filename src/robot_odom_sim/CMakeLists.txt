cmake_minimum_required(VERSION 3.0.2)
project(robot_odom_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/teleop_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
