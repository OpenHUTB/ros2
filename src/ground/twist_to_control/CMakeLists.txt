cmake_minimum_required(VERSION 3.0.2)
project(twist_to_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  nav_msgs
)

catkin_package()

# 三个脚本都要装：main.py 是入口，twist_core.py 是纯逻辑，twist_node.py 是节点
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/twist_core.py
  scripts/twist_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
