cmake_minimum_required(VERSION 3.0.2)
project(mrobot_teleop)

find_package(catkin REQUIRED)

catkin_package()

# 安装 launch 与可执行脚本
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/mrobot_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
