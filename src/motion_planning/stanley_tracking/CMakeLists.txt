cmake_minimum_required(VERSION 3.8)
project(stanley_tracking)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# simple_plot.py 与 sim_node.py 由三个模块共用，源码树里只有一份，放在
# src/motion_planning/common/。构建时拷进本包的 lib/<pkg>/，
# 这样各包仍可单独分发，仓库里不出现重复文件。
set(MOTION_COMMON "${CMAKE_CURRENT_SOURCE_DIR}/../common")

# 可执行脚本：带 shebang，用 install(PROGRAMS) 才会装上可执行位
install(PROGRAMS
  scripts/plan_control_node.py
  scripts/main.py
  main.sh
  ${MOTION_COMMON}/sim_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 被 import 的纯 Python 模块（无需可执行位）
install(FILES
  scripts/stanley.py
  scripts/simulate.py
  ${MOTION_COMMON}/simple_plot.py

  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md requirements.txt
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
