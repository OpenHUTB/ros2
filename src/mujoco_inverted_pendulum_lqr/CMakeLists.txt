cmake_minimum_required(VERSION 3.10)
project(mujoco_inverted_pendulum_lqr)

find_package(catkin REQUIRED)

catkin_package()

# 安装Python脚本
install(PROGRAMS
  main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装模型文件
install(FILES
  balance.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装launch文件夹
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
