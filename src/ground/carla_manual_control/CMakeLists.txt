cmake_minimum_required(VERSION 3.0.2)
project(carla_manual_control)

## Carla 键盘控制车辆
##
## main.py 是纯 Carla 客户端脚本，通过 launch 启动为 ROS 节点。
## 依赖：Carla Python API (hutb)、pygame

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  CATKIN_DEPENDS rospy
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch assets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)

#############
## Testing ##
#############
## 测试规则后续补充
