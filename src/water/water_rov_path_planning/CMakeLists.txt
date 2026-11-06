cmake_minimum_required(VERSION 3.8)
project(water_rov_path_planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# 核心库
add_library(rov_path_core STATIC
  src/heatmap.cpp
  src/ocean_current.cpp
  src/path_planner.cpp
)
target_include_directories(rov_path_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(rov_path_core
  ${OpenCV_LIBS}
  Eigen3::Eigen
)
ament_target_dependencies(rov_path_core
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs
  cv_bridge image_transport
)

# 主节点：热力图更新 + 洋流补偿 + 路径规划
add_executable(planning_node src/planning_node.cpp)
target_link_libraries(planning_node rov_path_core)
ament_target_dependencies(planning_node
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs
  cv_bridge image_transport
)

# 演示节点：合成海底环境 + 洋流场 + 目标点
add_executable(demo_node src/demo_node.cpp)
target_link_libraries(demo_node rov_path_core)
ament_target_dependencies(demo_node
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs
  cv_bridge image_transport
)

install(TARGETS
  rov_path_core
  planning_node
  demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_core test/test_core.cpp)
  if(TARGET test_core)
    target_link_libraries(test_core rov_path_core)
    ament_target_dependencies(test_core
      rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs
      cv_bridge image_transport
    )
  endif()
endif()

ament_package()
