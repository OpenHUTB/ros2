cmake_minimum_required(VERSION 3.0.2)
project(midterm_humanoid)

find_package(catkin REQUIRED)

catkin_package()

# 安装 Python 入口脚本，使 roslaunch 可按 pkg/type 找到 main.py
catkin_install_python(PROGRAMS main.py common.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# 安装 launch 文件
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
