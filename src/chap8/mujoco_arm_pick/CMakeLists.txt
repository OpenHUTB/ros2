cmake_minimum_required(VERSION 3.0.2)
project(mujoco_arm_pick)

find_package(catkin REQUIRED)

catkin_package()

# 纯 Python 包：无编译目标
# include_directories(${catkin_INCLUDE_DIRS})

# 只把入口脚本装到 devel/lib/<pkg>/，供 launch 以 type=main.py 启动
catkin_install_python(PROGRAMS
  main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch 文件安装到 share/<pkg>/launch/
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
