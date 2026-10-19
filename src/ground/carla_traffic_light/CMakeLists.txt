cmake_minimum_required(VERSION 3.0.2)
project(carla_traffic_light)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
