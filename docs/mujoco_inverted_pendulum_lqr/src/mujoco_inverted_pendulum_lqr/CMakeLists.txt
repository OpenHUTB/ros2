cmake_minimum_required(VERSION 3.10)
project(mujoco_inverted_pendulum_lqr)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  CATKIN_DEPENDS rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 安装Python脚本到bin目录
catkin_install_python(PROGRAMS
  src/mujoco_inverted_pendulum_lqr/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 【重点修改】将balance.xml模型文件，和脚本安装到同一个bin目录
install(FILES
  src/mujoco_inverted_pendulum_lqr/balance.xml
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
