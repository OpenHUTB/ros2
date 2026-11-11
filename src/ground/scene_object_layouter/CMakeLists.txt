cmake_minimum_required(VERSION 3.0.2)
project(scene_object_layouter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

# 纯 Python 包：三个脚本都装上，main.py 是入口
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/layouter_core.py
  scripts/layouter_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch 与 config 要装到 share 目录，否则 devel/install 空间找不到
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
