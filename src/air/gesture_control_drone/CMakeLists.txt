cmake_minimum_required(VERSION 3.0.2)
project(gesture_control_drone)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

# 所有 Python 脚本都会装到 ${CATKIN_PACKAGE_BIN_DESTINATION}（devel 空间里是
# 可执行副本），roslaunch/rosrun 才找得到节点。注意：辅助模块
# （camera_utils / gesture_detection / gesture_map / drone_client / gesture_gate）
# 也要一起装，否则节点 import 会失败。
# 只有"入口/节点"走 catkin_install_python：它在 devel 空间生成的是启动器，
# 被启动的文件能跑，但它生成的文件**不能当模块 import**。辅助模块必须用 install(FILES)。
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/gesture_ros_node.py
  scripts/drone_control_node.py
  scripts/gesture_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  scripts/camera_utils.py
  scripts/gesture_detection.py
  scripts/gesture_gate.py
  scripts/gesture_map.py
  scripts/drone_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
