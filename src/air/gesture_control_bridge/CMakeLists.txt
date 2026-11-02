cmake_minimum_required(VERSION 3.0.2)
project(gesture_control_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()

# 只有"入口/节点"交给 catkin_install_python：它在 devel 空间生成的是启动器
# （exec(compile(...))），不能当模块 import；辅助模块必须走 install(FILES)。
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/gesture_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  scripts/bridge_cmd.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
