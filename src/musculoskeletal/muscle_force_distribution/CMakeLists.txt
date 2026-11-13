cmake_minimum_required(VERSION 3.0.2)
project(muscle_force_distribution)

# 模块本身不依赖 rospy，本包只用来把它纳入 catkin 工作空间并支持 launch 启动
find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/force_alloc.py
  scripts/simulate.py
  scripts/simple_plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
