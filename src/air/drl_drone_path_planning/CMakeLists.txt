# Copyright (c) 2026 汤湘粤 (学号: 2535030067)
# SPDX-License-Identifier: MIT
# ROS 1 Noetic 功能包，使用 catkin 构建。

cmake_minimum_required(VERSION 3.0.2)
project(drl_drone_path_planning)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package()

# 入口脚本：catkin_install_python 会给它们包一层 wrapper。
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/train.py
  scripts/evaluate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 被 import 的模块必须按普通文件安装，否则 `import common` 拿到的是 wrapper，
# 命名空间里没有需要的函数（与 uav_ppo_nav 的处理方式一致）。
install(FILES
  scripts/common.py
  scripts/drone_env.py
  scripts/policy.py
  scripts/baseline.py
  scripts/drl_nav_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config models rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

# tests/ 下是纯 Python unittest（ROS 依赖由测试内的桩替代）。
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()
