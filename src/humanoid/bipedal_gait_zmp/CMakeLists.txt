cmake_minimum_required(VERSION 3.0.2)
project(bipedal_gait_zmp)

# 离线分析（scripts/main.py）只用 Python 标准库；scripts/gait_node.py 是本包的
# ROS 节点，把步态结果发布成话题，所以这里声明运行期需要的消息包。
find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
)

catkin_package()

# 安装清单必须与 scripts/ 下的实际文件一一对应：catkin_install_python 碰到
# 不存在的文件会在 configure 阶段直接报错，整个工作空间都编译不过。
# 包根 main.py 只供源码树里 `python3 main.py` 用，不重复安装（重名会冲突）。
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/gait_node.py
  scripts/animate.py
  scripts/lipm.py
  scripts/footstep.py
  scripts/gait.py
  scripts/dcm.py
  scripts/simple_plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
