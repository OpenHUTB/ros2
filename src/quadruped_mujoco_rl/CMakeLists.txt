cmake_minimum_required(VERSION 3.0.2)
project(quadruped_mujoco_rl)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main_train.py
  scripts/main_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)