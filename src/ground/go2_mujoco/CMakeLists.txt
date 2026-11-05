cmake_minimum_required(VERSION 3.0.2)
project(go2_mujoco)

find_package(catkin REQUIRED)

# catkin_package() 必须在使用 ${CATKIN_PACKAGE_*} 变量之前调用
catkin_package()

# install python package via setup.py (src/go2_mujoco/) so that
# `from go2_mujoco.common import ...` works under roslaunch
catkin_python_setup()

catkin_install_python(PROGRAMS src/go2_mujoco/main.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install launch files
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

