cmake_minimum_required(VERSION 3.0.2)
project(lateral_control_benchmark)

find_package(catkin REQUIRED)

catkin_package()

# Python 脚本：catkin 会按可执行文件安装，launch 才能按 type 找到它们
catkin_install_python(PROGRAMS
  scripts/lateral_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 离线入口也一并安装，方便在 install 空间直接跑对比实验
catkin_install_python(PROGRAMS
  main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
