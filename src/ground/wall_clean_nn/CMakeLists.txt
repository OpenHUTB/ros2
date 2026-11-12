cmake_minimum_required(VERSION 3.0.2)
project(wall_clean_nn)

find_package(catkin REQUIRED)

catkin_package()

# 节点入口
catkin_install_python(PROGRAMS
  nodes/wall_clean_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 运行期脚本：节点用 rospkg 定位包路径后导入，不依赖源码树相对路径
install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE
)

catkin_install_python(PROGRAMS
  scripts/train.py
  scripts/evaluate.py
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY <dir>/) 的尾斜杠会把内容平铺，必须分别装到子目录
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
  PATTERN "__pycache__" EXCLUDE
)
