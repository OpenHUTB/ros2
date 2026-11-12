cmake_minimum_required(VERSION 3.0.2)
project(lateral_control_benchmark)

find_package(catkin REQUIRED)

# 把 lateral_control_benchmark 这个 Python 包安装到 devel/lib/python3/dist-packages，
# 否则 roslaunch 启动节点脚本时会 import 不到（ROS 1 的必须步骤）
catkin_python_setup()

catkin_package()

# 节点脚本：必须带可执行位，否则 devel 空间的符号链接无法执行
catkin_install_python(PROGRAMS
  scripts/lateral_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 说明：离线基准入口 main.py 不在此安装。它是"在源码目录直接运行"的工具
# （python3 main.py --mode bench），依赖同目录下的 Python 包；
# 安装到 devel 空间后路径变化，反而会导致 import 失败。
