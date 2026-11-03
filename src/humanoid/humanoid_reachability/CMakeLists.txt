cmake_minimum_required(VERSION 3.8)
project(humanoid_reachability)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 运动学与可达域统计库
add_library(reachability_lib SHARED
  src/kinematics.cpp
  src/workspace_analyzer.cpp
)
target_include_directories(reachability_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(reachability_lib
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)
# Eigen3 不是 ament 包，必须直接链接 CMake target 才能传递头文件路径
target_link_libraries(reachability_lib Eigen3::Eigen)

# 可达域分析节点（main.cpp）
add_executable(humanoid_reachability_node src/main.cpp)
ament_target_dependencies(humanoid_reachability_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)
target_link_libraries(humanoid_reachability_node reachability_lib Eigen3::Eigen)

# 双臂关节演示节点
add_executable(arm_demo_node src/arm_demo_node.cpp)
ament_target_dependencies(arm_demo_node
  rclcpp
  std_msgs
)
target_link_libraries(arm_demo_node reachability_lib Eigen3::Eigen)

install(TARGETS reachability_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  humanoid_reachability_node
  arm_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(reachability_lib)
ament_export_dependencies(
  rclcpp std_msgs geometry_msgs visualization_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
