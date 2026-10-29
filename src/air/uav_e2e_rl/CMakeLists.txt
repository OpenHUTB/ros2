cmake_minimum_required(VERSION 3.0.2)
project(uav_e2e_rl)

find_package(catkin REQUIRED COMPONENTS
  rospy roscpp std_msgs sensor_msgs geometry_msgs nav_msgs cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/train.py
  scripts/deploy_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
