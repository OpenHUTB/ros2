cmake_minimum_required(VERSION 3.0.2)
project(carla_traffic_light)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

# 包根 main.py 是源码树里 `python3 main.py` 的入口，转调 scripts/main.py。
# 安装时只装 scripts/main.py（原名安装到 bin），两者同名，装包根版本会冲突，
# 所以这里保持只装一份；rosrun / roslaunch 与直接运行源码树的入口行为一致。
install(PROGRAMS
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
