cmake_minimum_required(VERSION 3.8)
project(mujoco_slope_car)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS main.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(FILES scene.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
