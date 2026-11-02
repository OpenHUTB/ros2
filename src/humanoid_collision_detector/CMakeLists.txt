cmake_minimum_required(VERSION 3.8)
project(humanoid_collision_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# collision detector library
add_library(collision_detector_lib SHARED
  humanoid_collision_detector/collision_detector_core.cpp
)
ament_target_dependencies(collision_detector_lib
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)
# Eigen3 不是 ament 包，必须直接链接 CMake target 才能传递头文件路径
target_link_libraries(collision_detector_lib Eigen3::Eigen)
target_include_directories(collision_detector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# main node
add_executable(collision_detector_node
  humanoid_collision_detector/collision_detector_node.cpp
)
ament_target_dependencies(collision_detector_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)
target_link_libraries(collision_detector_node collision_detector_lib Eigen3::Eigen)

# obstacle demo node
add_executable(obstacle_demo_node
  humanoid_collision_detector/obstacle_demo_node.cpp
)
ament_target_dependencies(obstacle_demo_node
  rclcpp
  geometry_msgs
)

# install：库装到 lib，可执行文件装到 lib/${PROJECT_NAME}（ROS 2 约定）
install(TARGETS collision_detector_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
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

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# export
ament_export_include_directories(include)
ament_export_libraries(collision_detector_lib)
ament_export_dependencies(
  rclcpp std_msgs geometry_msgs visualization_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
