cmake_minimum_required(VERSION 3.0.2)
project(uav_nn_perception_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs std_srvs sensor_msgs geometry_msgs nav_msgs tf tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/airsim_bridge.py
  scripts/keyboard_teleop.py
  scripts/uav_features.py
  scripts/lidar_perception.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
