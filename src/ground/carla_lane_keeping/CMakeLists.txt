cmake_minimum_required(VERSION 3.0.2)
project(carla_lane_keeping)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()

# 包根 main.py 是源码树里 `python3 main.py` 的入口，转调 scripts/main.py。
# 安装时只装 scripts/main.py（原名安装到 bin），两者同名，装包根版本会冲突，
# 所以这里保持只装一份；rosrun 与直接运行源码树的入口行为一致。
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/lane_detection.py
  scripts/lane_detector.py
  scripts/lane_keeping_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# tools/ 下的两个脚本是**离线验证工具**（需要本机 CARLA 与 carla 包），
# 不参与安装，也不进 source 空间，所以这里不列。
install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
