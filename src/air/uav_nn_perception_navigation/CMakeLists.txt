cmake_minimum_required(VERSION 3.0.2)
project(uav_nn_perception_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs std_srvs sensor_msgs geometry_msgs nav_msgs tf tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/airsim_bridge.py
  scripts/keyboard_teleop.py
  scripts/lidar_perception.py
  scripts/trajectory_control.py
  scripts/eval_multi_target.py
  scripts/camera_perception.py
  scripts/end2end_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 被 import 的模块不是可执行入口：若放进 catkin_install_python，
# catkin 会为它生成 wrapper，import 拿到的是 wrapper 而不是源码。
# train_control 被 train_end2end import，train_end2end 又被 end2end_node import。
install(FILES
  scripts/uav_features.py
  scripts/safety_shield.py
  scripts/train_control.py
  scripts/train_end2end.py
  scripts/scene_image.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_uav_features.py)
  catkin_add_nosetests(test/test_safety_shield.py)
  catkin_add_nosetests(test/test_scene_image.py)
endif()

install(DIRECTORY launch config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
