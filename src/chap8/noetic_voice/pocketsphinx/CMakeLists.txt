cmake_minimum_required(VERSION 3.0.2)
project(pocketsphinx)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS nodes/recognizer.py nodes/voice_cmd_vel.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY demo DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
