cmake_minimum_required(VERSION 3.0.2)
project(midterm_humanoid)

find_package(catkin REQUIRED)

# catkin_package() 必须在使用 ${CATKIN_PACKAGE_*} 变量之前调用
catkin_package()

# 通过 setup.py 安装 Python 包（src/midterm_humanoid/），使
# `from midterm_humanoid.common import ...` 在 roslaunch 下可导入
catkin_python_setup()

catkin_install_python(PROGRAMS src/midterm_humanoid/main.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# 安装 launch 文件
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)

