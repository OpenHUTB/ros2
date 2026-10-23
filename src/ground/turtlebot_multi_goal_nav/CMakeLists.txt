cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_multi_goal_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslaunch
)

catkin_package(CATKIN_DEPENDS rospy)

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()

# 将 main.py 安装为可执行节点（入口 main.py，可由 roslaunch / rosrun 调用）
install(PROGRAMS main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
