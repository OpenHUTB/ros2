cmake_minimum_required(VERSION 3.0.2)
project(uav_neural_perception)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs sensor_msgs geometry_msgs cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/dataset.py
  scripts/model.py
  scripts/train.py
  scripts/perception_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
