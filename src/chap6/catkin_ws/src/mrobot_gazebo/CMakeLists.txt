cmake_minimum_required(VERSION 3.0.2)
project(mrobot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  roscpp
  rospy
)

catkin_package(
  CATKIN_DEPENDS gazebo_plugins gazebo_ros gazebo_ros_control roscpp rospy
)

include_directories(${catkin_INCLUDE_DIRS})

# 安装 launch、urdf、worlds、models 到功能包 share 目录
install(DIRECTORY launch urdf worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
