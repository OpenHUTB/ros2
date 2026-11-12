cmake_minimum_required(VERSION 3.0.2)
project(neural_ca_drone_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs sensor_msgs nav_msgs geometry_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/sim_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config worlds urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
