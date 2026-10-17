cmake_minimum_required(VERSION 3.8)
project(auv_end2end)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 安装Python节点
install(PROGRAMS
  auv_end2end/auv_obstacle_avoid.py
  auv_end2end/auv_vel_ctrl.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装仿真世界文件
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# 安装launch启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
