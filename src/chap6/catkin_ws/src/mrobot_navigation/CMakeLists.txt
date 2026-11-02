cmake_minimum_required(VERSION 3.0.2)
project(mrobot_navigation)

find_package(catkin REQUIRED)

catkin_package()

# 安装 launch、rviz、maps 到功能包 share 目录
install(DIRECTORY launch rviz maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
