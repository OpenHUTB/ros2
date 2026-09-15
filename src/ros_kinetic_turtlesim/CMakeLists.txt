cmake_minimum_required(VERSION 2.8.3)
project(ros_kinetic_turtlesim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  turtlesim
)

catkin_package()

# 安装 Python 入口脚本 main.py。
# 注意：clone 后需保证脚本有可执行权限（chmod +x main.py），
# Kinetic 下使用系统 Python 2.7 解释器，脚本本身兼容 Python 2/3。
catkin_install_python(PROGRAMS
  main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
