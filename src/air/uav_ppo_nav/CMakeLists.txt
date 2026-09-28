cmake_minimum_required(VERSION 3.0.2)
project(uav_ppo_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
)

catkin_package()

# 所有 Python 脚本装到同一个 lib 目录，保证 `from policy import ...` 可导入
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/ppo_nav_node.py
  scripts/policy.py
  scripts/env.py
  scripts/train.py
  scripts/eval.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
