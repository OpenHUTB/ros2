cmake_minimum_required(VERSION 3.0.2)
project(ground)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

# 如果你有其他的 Python 脚本，可以继续加
catkin_install_python(PROGRAMS
  cliff_brake.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
