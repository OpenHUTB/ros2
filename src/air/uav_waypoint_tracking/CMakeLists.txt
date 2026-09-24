cmake_minimum_required(VERSION 3.0.2)
project(uav_waypoint_tracking)

## 本包为纯 Python 包，无需编译 C++，包含三个脚本：
##   - pid_tracker.py      自研 PID 航点跟踪节点（主节点）
##   - compare_builtin.py  用 AirSim 内置位置接口跑同一组航点（对照组）
##   - plot_tracking.py    读取 CSV 日志，绘制轨迹对比图与误差曲线

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf2_ros
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy std_msgs tf2_ros visualization_msgs
)

###########
## Install ##
###########

## Python 脚本
catkin_install_python(PROGRAMS
  scripts/pid_tracker.py
  scripts/compare_builtin.py
  scripts/plot_tracking.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 参数、launch 与 RViz 配置
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)