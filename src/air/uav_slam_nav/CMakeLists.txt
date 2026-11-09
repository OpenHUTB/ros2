cmake_minimum_required(VERSION 3.0.2)
project(uav_slam_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  move_base_msgs
  tf2_ros
  tf2_geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/points_to_scan.py
  scripts/nav_relay.py
  scripts/simple_nav.py
  scripts/demo.py
  scripts/nav_logic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
