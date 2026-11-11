cmake_minimum_required(VERSION 3.0.2)
project(mujoco_slope_car)
find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES scene.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS main.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
