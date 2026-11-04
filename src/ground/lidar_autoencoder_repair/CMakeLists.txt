cmake_minimum_required(VERSION 3.0.2)
project(lidar_autoencoder_repair)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs sensor_msgs
)

catkin_install_python(PROGRAMS
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 训练脚本和模型权重不安装到 bin，由用户手动运行训练
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
