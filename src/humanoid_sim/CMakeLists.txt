cmake_minimum_required(VERSION 3.0.2)
project(humanoid_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs geometry_msgs nav_msgs sensor_msgs tf2_ros
)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/keyboard_teleop.py
  scripts/plot_viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
