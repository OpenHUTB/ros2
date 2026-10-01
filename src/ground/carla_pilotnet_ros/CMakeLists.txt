cmake_minimum_required(VERSION 3.0.2)
project(carla_pilotnet_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy sensor_msgs std_msgs geometry_msgs
)
catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs geometry_msgs
)

catkin_install_python(PROGRAMS
  scripts/image_publisher.py
  scripts/pilotnet_inference.py
  scripts/control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config assets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
