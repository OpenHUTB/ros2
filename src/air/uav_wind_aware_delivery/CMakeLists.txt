cmake_minimum_required(VERSION 3.0.2)
project(uav_wind_aware_delivery)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/nodes/sim_node.py
  scripts/nodes/control_node.py
  scripts/nodes/mission_node.py
  scripts/nodes/predictor_node.py
  scripts/nodes/viewer_node.py
  scripts/nodes/teleop_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 节点公共引导模块：6 个节点都会 import _bootstrap，必须和节点一起安装到 lib/<pkg>。
install(FILES scripts/nodes/_bootstrap.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# 运行期 Python 包：安装空间下由 ROS 的 PYTHONPATH 提供 import。
install(DIRECTORY src/uav_wind_aware_delivery/
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
        PATTERN "__pycache__" EXCLUDE)

# main.py / main.sh 以及它们调用的 tools/tests/scripts；
# main.py 会用 rospkg 从安装路径回到 share/<pkg>，所以这些辅助目录也要安装。
catkin_install_python(PROGRAMS
  main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS main.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY tools tests scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "__pycache__" EXCLUDE)

install(DIRECTORY models launch results
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
