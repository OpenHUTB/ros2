cmake_minimum_required(VERSION 3.0.2)
project(uav_wind_aware_delivery)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/nodes/sim_node.py
  scripts/nodes/control_node.py
  scripts/nodes/mission_node.py
  scripts/nodes/predictor_node.py
  scripts/nodes/viewer_node.py
  scripts/nodes/teleop_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models launch results
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
