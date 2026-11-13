cmake_minimum_required(VERSION 3.0.2)
project(rov_thruster_allocation_ros1)

# 分配算法使用 numpy，ROS Noetic 的 python3-numpy 由系统提供
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  main.py
  allocation.py
  offline_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
