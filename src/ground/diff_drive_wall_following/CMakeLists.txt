cmake_minimum_required(VERSION 3.0.2)
project(diff_drive_wall_following)

# 本模块使用 rospy、geometry_msgs（速度指令）和 sensor_msgs（激光数据）
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  main.py
  wall_follower.py
  offline_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
