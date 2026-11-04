cmake_minimum_required(VERSION 3.0.2)
project(shadow_hand_mujoco)

find_package(catkin REQUIRED)

# install python package via setup.py (src/shadow_hand_mujoco/) so that
# `from shadow_hand_mujoco.common import ...` works under roslaunch
catkin_python_setup()

catkin_install_python(PROGRAMS src/shadow_hand_mujoco/main.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install launch files
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_package()
