cmake_minimum_required(VERSION 3.0.2)
project(diff_robot)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch urdf worlds scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
