cmake_minimum_required(VERSION 3.8)
project(rov_wreck_inspection)

find_package(ament_cmake REQUIRED)

# 纯 Python 节点，运行时依赖见 package.xml
install(PROGRAMS
  scripts/hull_detector.py
  scripts/inspection_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch world urdf config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
