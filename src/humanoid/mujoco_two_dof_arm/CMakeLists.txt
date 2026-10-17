cmake_minimum_required(VERSION 3.0.2)
project(mujoco_arm_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/arm_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
