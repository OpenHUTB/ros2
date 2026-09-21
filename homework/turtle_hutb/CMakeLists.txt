cmake_minimum_required(VERSION 3.0.2)

project(turtle_hutb)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  turtlesim
  std_srvs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/draw_openhutb.py
    scripts/draw_face.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
