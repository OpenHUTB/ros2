cmake_minimum_required(VERSION 3.0.2)
project(panda_keyboard_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  moveit_commander
  moveit_ros_planning_interface
)

catkin_package()
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/panda_keyboard_control.py
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
