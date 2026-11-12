cmake_minimum_required(VERSION 3.8)
project(fibonacci_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成自定义 action 接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)

# 安装 Python 包
ament_python_install_package(${PROJECT_NAME})

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装入口脚本（如果需要）
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
