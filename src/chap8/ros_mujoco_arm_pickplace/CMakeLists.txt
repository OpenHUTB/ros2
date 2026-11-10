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
  scripts/pickplace_planner.py
  scripts/cubic_interpolator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch / 模型 / RViz 配置安装到 share/<pkg>/
# 注意：rviz 配置在包根目录（rviz/ur5e.rviz），安装后位于 share/<pkg>/rviz/
install(DIRECTORY
  launch
  models
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
