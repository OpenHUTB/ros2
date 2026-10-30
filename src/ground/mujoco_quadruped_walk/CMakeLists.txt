cmake_minimum_required(VERSION 3.8)
project(mujoco_quadruped_walk)

find_package(ament_cmake REQUIRED)

install(DIRECTORY scripts models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
