cmake_minimum_required(VERSION 3.0.2)
project(robot_voice)
find_package(catkin REQUIRED COMPONENTS geometry_msgs rospy std_msgs)
catkin_package(CATKIN_DEPENDS geometry_msgs rospy std_msgs)
catkin_install_python(PROGRAMS scripts/voice_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
