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
  scripts/train_control.py
  scripts/trajectory_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_uav_features.py)
endif()

install(DIRECTORY launch config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
