cmake_minimum_required(VERSION 3.0.2)
project(vehicle_pygame_teleop)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

catkin_install_python(PROGRAMS
  scripts/vehicle_pygame_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
