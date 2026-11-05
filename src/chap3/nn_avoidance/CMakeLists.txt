 cmake_minimum_required(VERSION 3.0.2)
project(nn_avoidance)
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
  sensor_msgs
  geometry_msgs
)
catkin_package(
  CATKIN_DEPENDS std_msgs rospy sensor_msgs geometry_msgs rospkg
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  nn_avoidance/infer_node.py
  nn_avoidance/train.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)