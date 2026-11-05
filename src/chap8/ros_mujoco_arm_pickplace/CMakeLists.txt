cmake_minimum_required(VERSION 3.0.2)
project(ros_mujoco_arm_pickplace)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs
)

# 纯 Python 包：无编译目标
# include_directories(${catkin_INCLUDE_DIRS})

# 节点脚本 + 依赖模块一起安装到 devel/lib/<pkg>/
catkin_install_python(PROGRAMS
  scripts/main_mujoco_bridge.py
  scripts/main_pick_fsm.py
  scripts/pickplace_core.py
  scripts/cubic_interpolator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch 与模型文件安装到 share/<pkg>/
install(DIRECTORY
  launch
  models
  src/rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
