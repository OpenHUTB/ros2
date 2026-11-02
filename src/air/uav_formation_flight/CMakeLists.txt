cmake_minimum_required(VERSION 3.0.2)
project(uav_formation_flight)

## 本包为纯 Python 包，无需编译 C++，包含三个脚本：
##   - formation_controller.py  Leader-Follower 编队控制主节点
##   - plot_formation.py        读取 CSV 日志，绘制队形误差与航迹图
##   - fleet_check.py           多机连接自检（列出载具、逐架起降验证）

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy std_msgs visualization_msgs
)

###########
## Install ##
###########

## Python 脚本
catkin_install_python(PROGRAMS
  scripts/formation_controller.py
  scripts/plot_formation.py
  scripts/fleet_check.py
  scripts/tune_hover.py
  scripts/deploy_probe.py
  scripts/lag_probe.py
  scripts/vehicle_diag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 参数与 launch
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
