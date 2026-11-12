cmake_minimum_required(VERSION 3.0.2)
project(uav_perception_trajectory)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/perception.py
  scripts/trajectory.py
  scripts/controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
