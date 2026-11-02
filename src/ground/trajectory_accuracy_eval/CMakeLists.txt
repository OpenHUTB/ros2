cmake_minimum_required(VERSION 3.0.2)
project(trajectory_accuracy_eval)

find_package(catkin REQUIRED)

# 把 traj_eval 这个 Python 包装到 devel/lib/python3/dist-packages，
# 否则 roslaunch 启动 scripts/ 下的节点时 import 不到（ROS 1 必须步骤）
catkin_python_setup()

catkin_package()

# 两个节点脚本都需要可执行位，否则 devel 空间的软链无法执行
catkin_install_python(PROGRAMS
  scripts/odom_eval_node.py
  scripts/sim_odom_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 说明：离线入口 main.py 不在安装之列。它是"在源码目录直接运行"的工具
# （python3 main.py --mode demo），依赖同目录下的 traj_eval 包；
# 安装到 devel 空间后路径变化反而会导致 import 失败。
