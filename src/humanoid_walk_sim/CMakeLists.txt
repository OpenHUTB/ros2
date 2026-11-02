cmake_minimum_required(VERSION 3.0.2)
project(humanoid_walk_sim)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

catkin_install_python(PROGRAMS
  scripts/cmd_vel_publisher.py
  scripts/odom_sim.py
  scripts/robot_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
