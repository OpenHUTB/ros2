cmake_minimum_required(VERSION 3.0.2)
project(uav_takeoff_hover_land)

# 本模块只使用 rospy 与 geometry_msgs，不需要编译 C++ 代码
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_package()

# 安装 ROS 节点入口、状态机模块和离线仿真脚本
catkin_install_python(PROGRAMS
  main.py
  flight_state_machine.py
  offline_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
