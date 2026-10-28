cmake_minimum_required(VERSION 3.0.2)
project(vehicle_rviz_sim)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
