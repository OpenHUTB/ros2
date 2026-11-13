cmake_minimum_required(VERSION 3.8)
project(fibonacci_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成自定义 action 接口（Goal / Result / Feedback 三段）
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)

# 安装入口脚本。源文件位于 Python 包目录下，安装时用 RENAME 去掉 .py 后缀，
# 安装结果为 lib/fibonacci_action/server 与 lib/fibonacci_action/client，
# 这样 `ros2 run fibonacci_action server|client` 才能找到可执行文件。
install(PROGRAMS
  fibonacci_action/server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME server
)
install(PROGRAMS
  fibonacci_action/client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME client
)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
