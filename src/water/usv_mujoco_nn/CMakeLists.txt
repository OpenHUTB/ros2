cmake_minimum_required(VERSION 3.0.2)
project(usv_mujoco_nn)

find_package(catkin REQUIRED)

catkin_package()

# 节点入口：安装到 lib/<包>/，由 roslaunch 直接调用
catkin_install_python(PROGRAMS
  nodes/usv_nn_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 节点运行期依赖的模块（sim_core / occupancy / oracle / policy / nn）：
# 安装到 share/<包>/scripts，节点用 rospkg 定位包路径后导入，
# 这样安装空间下不依赖源码树的相对路径。
install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE
)

# 可直接在安装空间运行的脚本
catkin_install_python(PROGRAMS
  scripts/train.py
  scripts/evaluate.py
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 注意：install(DIRECTORY <dir>/) 带尾斜杠会把目录**内容**平铺到目标目录，
# 因此 launch、config、models 必须分别安装到各自的子目录，
# 否则节点按 models/ 查找权重会失败。
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
