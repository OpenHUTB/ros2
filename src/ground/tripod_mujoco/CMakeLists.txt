cmake_minimum_required(VERSION 3.0.2)
project(tripod_mujoco)

find_package(catkin REQUIRED)

# install python package via setup.py (src/tripod_mujoco/) so that
# `from tripod_mujoco.common import ...` works under roslaunch
catkin_python_setup()

catkin_install_python(PROGRAMS src/tripod_mujoco/main.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install launch files
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_package()
