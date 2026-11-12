cmake_minimum_required(VERSION 3.0.2)
project(speed_profile_check)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/speed_core.py
  scripts/speed_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
