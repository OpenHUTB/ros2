cmake_minimum_required(VERSION 3.0.2)
project(ackermann_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/bike_model.py
  scripts/sim_node.py
  scripts/keyboard_teleop.py
  scripts/plot_viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
