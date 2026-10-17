cmake_minimum_required(VERSION 3.0.2)
project(carla_lane_keeping)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/lane_detector.py
  scripts/lane_keeping_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
