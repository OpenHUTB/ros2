cmake_minimum_required(VERSION 3.0.2)
project(uav_lidar_dqn_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/drone_bridge.py
  scripts/teleop_keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config models maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
