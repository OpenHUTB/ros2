cmake_minimum_required(VERSION 3.0.2)
project(uav_landing_monitor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/landing_monitor_node.py
  scripts/monitor_core.py
  tools/plot_report.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
