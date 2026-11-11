cmake_minimum_required(VERSION 3.10)
project(web_remote_car)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  gazebo_ros
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy roscpp gazebo_ros geometry_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch urdf www
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
    PATTERN "*.launch"
    PATTERN "*.xacro"
    PATTERN "*.html"
    PATTERN "*.css"
    PATTERN "*.js"
)

# 前端静态页面服务节点，被 launch/web_bridge.launch 调用
catkin_install_python(PROGRAMS
  scripts/web_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

