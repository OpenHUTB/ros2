cmake_minimum_required(VERSION 3.0.2)
project(carlair_ros_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/odom_pub.py
  scripts/cmd_vel_sub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
