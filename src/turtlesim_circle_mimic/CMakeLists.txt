cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_circle_mimic)

find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS main.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
