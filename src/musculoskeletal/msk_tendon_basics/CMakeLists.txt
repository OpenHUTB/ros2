cmake_minimum_required(VERSION 3.0.2)
project(msk_tendon_basics)

# 模型与实验都是纯 Python，不依赖 rospy；catkin 只负责统一入口和 launch。
find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/muscle_tendon.py
  scripts/joint_tendon.py
  scripts/experiments.py
  scripts/plots.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
