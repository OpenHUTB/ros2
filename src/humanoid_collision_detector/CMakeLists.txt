cmake_minimum_required(VERSION 3.8)
project(humanoid_collision_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# collision detector library
add_library(collision_detector_lib SHARED
  src/collision_detector_core.cpp
)
ament_target_dependencies(collision_detector_lib
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  shape_msgs
  tf2
  tf2_ros
  tf2_eigen
  Eigen3
)
target_include_directories(collision_detector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# main node
add_executable(collision_detector_node
  src/collision_detector_node.cpp
)
ament_target_dependencies(collision_detector_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  shape_msgs
  tf2
  tf2_ros
  tf2_eigen
  Eigen3
)
target_link_libraries(collision_detector_node collision_detector_lib)

# obstacle demo node
add_executable(obstacle_demo_node
  src/obstacle_demo_node.cpp
)
ament_target_dependencies(obstacle_demo_node
  rclcpp
  geometry_msgs
)

# install
install(TARGETS
  collision_detector_lib
  collision_detector_node
  obstacle_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# export
ament_export_include_directories(include)
ament_export_libraries(collision_detector_lib)
ament_export_dependencies(
  rclcpp std_msgs sensor_msgs geometry_msgs
  visualization_msgs shape_msgs tf2 tf2_ros tf2_eigen urdf Eigen3
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
