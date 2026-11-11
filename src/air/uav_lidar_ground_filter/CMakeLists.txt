cmake_minimum_required(VERSION 3.0.2)
project(uav_lidar_ground_filter)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/ground_filter_node.py
  scripts/ground_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
