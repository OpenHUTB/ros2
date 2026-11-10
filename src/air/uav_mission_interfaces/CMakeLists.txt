cmake_minimum_required(VERSION 3.8)
project(uav_mission_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteMission.action"
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
ament_package()
