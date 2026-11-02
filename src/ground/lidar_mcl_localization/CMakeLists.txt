cmake_minimum_required(VERSION 3.0.2)
project(lidar_mcl_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/map_publisher.py
  scripts/mcl_node.py
  scripts/teleop.py
  scripts/goal_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch maps urdf worlds config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
