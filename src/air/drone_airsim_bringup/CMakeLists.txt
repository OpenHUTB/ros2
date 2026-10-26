cmake_minimum_required(VERSION 3.0.2)
project(drone_airsim_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/teleop.py
  scripts/collect_dataset.py
  scripts/train_yolo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
