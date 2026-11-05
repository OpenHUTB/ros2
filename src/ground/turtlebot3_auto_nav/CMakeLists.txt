cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_auto_nav)

## TurtleBot3 自主导航功能包
## 包含：launch 启动文件、导航参数配置、RViz 配置、静态地图

find_package(catkin REQUIRED)

catkin_package()

#############
## Install ##
#############

install(DIRECTORY launch param maps rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
