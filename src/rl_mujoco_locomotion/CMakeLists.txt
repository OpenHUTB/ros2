cmake_minimum_required(VERSION 3.8)
project(rl_mujoco_locomotion)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  main.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  assets
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
