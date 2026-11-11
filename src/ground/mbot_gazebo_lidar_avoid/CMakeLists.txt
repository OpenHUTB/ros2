cmake_minimum_required(VERSION 3.0.2)
project(mbot_gazebo_lidar_avoid)

find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs sensor_msgs)

catkin_package()

# Python 节点安装到 lib/<pkg>/，并保留可执行权限
catkin_install_python(
  PROGRAMS scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# 资源文件必须安装，否则 catkin_make install 后 roslaunch 找不到
install(DIRECTORY launch config urdf worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# README 是源码级说明，不装进 install 空间；LICENSE 需随包分发
install(FILES LICENSE
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
