cmake_minimum_required(VERSION 3.8)
project(cartpole_lqr)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# 可执行脚本：带 shebang，用 install(PROGRAMS) 才会装上可执行位，
# 才能被 ros2 run / launch 的 exec 直接调用。
install(PROGRAMS
  scripts/ros_node.py
  scripts/main.py

  DESTINATION lib/${PROJECT_NAME}
)

# 被 import 的纯 Python 模块（无 shebang，不需要可执行位），
# 与上面安装到同一目录，保证 ros_node.py 能 import 到它们。
install(FILES
  scripts/cartpole.py
  scripts/minilinalg.py
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
