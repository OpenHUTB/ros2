cmake_minimum_required(VERSION 3.8)
project(ackermann_dubins_path)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 可执行脚本：带 shebang，用 install(PROGRAMS) 才会装上可执行位
install(PROGRAMS
  scripts/ros_node.py
  scripts/main.py
  DESTINATION lib/${PROJECT_NAME}
)

# 被 import 的纯 Python 模块（无需可执行位），装到同一目录保证 import 可解析
install(FILES
  scripts/dubins.py
  scripts/simulate.py
  scripts/simple_plot.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md requirements.txt
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
