cmake_minimum_required(VERSION 3.0.2)
project(braking_distance_check)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/braking_core.py
  scripts/braking_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
