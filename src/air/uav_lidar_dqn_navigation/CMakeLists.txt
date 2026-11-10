cmake_minimum_required(VERSION 3.0.2)
project(uav_lidar_dqn_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/drone_bridge.py
  scripts/teleop_keyboard.py
  scripts/lidar_scan_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# lidar_features 是被 import 的模块，不是可执行入口：
# 若放进 catkin_install_python，catkin 会为它生成 wrapper，
# `from lidar_features import ...` 实际拿到的是 wrapper 而不是源码。
install(FILES
  scripts/lidar_features.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # 指向具体测试文件：部分环境里 nosetests 对目录收集会漏掉用例
  catkin_add_nosetests(test/test_lidar_features.py)
endif()

install(DIRECTORY launch config models maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
