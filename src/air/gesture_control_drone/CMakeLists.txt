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
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/gesture_ros_node.py
  scripts/drone_control_node.py
  scripts/gesture_demo.py
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
