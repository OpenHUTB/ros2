cmake_minimum_required(VERSION 3.0.2)
project(carla_nn_scan_matching)

find_package(catkin REQUIRED COMPONENTS
  rospy sensor_msgs nav_msgs geometry_msgs tf tf2_ros
)
catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs nav_msgs geometry_msgs tf tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/lidar_simulator.py
  scripts/nn_odometry.py
  scripts/mapper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)

# 安装模型权重
install(FILES scan_match_model.pth
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
