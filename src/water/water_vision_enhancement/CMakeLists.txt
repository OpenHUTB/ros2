cmake_minimum_required(VERSION 3.8)
project(water_vision_enhancement)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 图像增强核心库
add_library(image_enhancer SHARED
  src/image_enhancer.cpp
)
ament_target_dependencies(image_enhancer
  rclcpp
  OpenCV
)
target_link_libraries(image_enhancer ${OpenCV_LIBS})

# 目标追踪核心库
add_library(object_tracker SHARED
  src/object_tracker.cpp
)
ament_target_dependencies(object_tracker
  rclcpp
  OpenCV
)
target_link_libraries(object_tracker ${OpenCV_LIBS})

# 主节点：图像增强 + 目标追踪
add_executable(vision_node
  src/vision_node.cpp
)
ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  OpenCV
)
target_link_libraries(vision_node
  image_enhancer
  object_tracker
  ${OpenCV_LIBS}
)

# 演示节点：模拟浑浊水体视频输入
add_executable(video_demo_node
  src/video_demo_node.cpp
)
ament_target_dependencies(video_demo_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)
target_link_libraries(video_demo_node ${OpenCV_LIBS})

# 安装
# 共享库必须安装到标准 lib 目录并导出，否则节点启动时
# 动态链接器找不到 libimage_enhancer.so（exit code 127）
install(TARGETS
  image_enhancer
  object_tracker
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  vision_node
  video_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# 资源文件标记
install(FILES resource/water_vision_enhancement
  DESTINATION share/${PROJECT_NAME}
)

# 文档
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
