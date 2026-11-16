cmake_minimum_required(VERSION 3.0.2)
project(uav_openhutb_lidar_nn_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs nav_msgs sensor_msgs std_srvs tf2_ros
)

catkin_install_python(PROGRAMS
  main.py
  scripts/airsim_bridge.py
  scripts/mock_sim.py
  scripts/keyboard_control.py
  scripts/lidar_perception.py
  scripts/grid_nav.py
  scripts/nn_policy.py
  scripts/nn_train.py
  scripts/dataset_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
