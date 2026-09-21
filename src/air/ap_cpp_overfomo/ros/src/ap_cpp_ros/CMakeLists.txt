cmake_minimum_required(VERSION 3.0.2)
project(ap_cpp_ros)

## The package is pure Python: rospy nodes plus launch and RViz config.  There
## is nothing to compile, but the standard catkin incantation is kept so the
## package behaves like any other in a workspace.

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
)

catkin_package()

## Install the nodes as executables on the catkin bin path.
catkin_install_python(PROGRAMS
  scripts/ap_cpp_rviz_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## No catkin_add_nosetests here on purpose: the node imports rospy at module
## scope, so anything importing it needs a sourced ROS environment, and the
## planner logic it drives is already covered by the repository-level suite
## (`python -m unittest discover -s tests`), which needs no ROS at all.
