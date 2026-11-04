cmake_minimum_required(VERSION 3.0.2)
project(ros_scenario_runner)

## 纯 Python 包：不需要 find_package 的 COMPONENTS 也能编译，
## 但显式声明依赖能让 catkin 检查出 topic 类型用到的消息包是否缺失。
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/scenario_core.py
  scripts/scenario_runner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
