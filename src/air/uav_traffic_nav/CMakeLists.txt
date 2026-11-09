cmake_minimum_required(VERSION 3.0.2)
project(uav_traffic_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package()

# 入口脚本（可执行，catkin 会给它们包一层 wrapper）
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/train.py
  scripts/eval.py
  scripts/traffic_nav_node.py
  scripts/traffic_feed_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 被 import 的模块：必须以普通文件安装（理由同 uav_ppo_nav）
install(FILES
  scripts/policy.py
  scripts/traffic_env.py
  scripts/baselines.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config models results
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
