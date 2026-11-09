cmake_minimum_required(VERSION 3.0.2)
project(diff_drive_odom_calib)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/sim_node.py
  scripts/teleop.py
  scripts/demo_drive.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# models/ 必须一起装：config.py 通过 PACKAGE_ROOT/models/diff_drive.xml 找场景，
# 漏装的话 catkin_make install 之后找不到模型（评审指出过）。
install(DIRECTORY launch config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
