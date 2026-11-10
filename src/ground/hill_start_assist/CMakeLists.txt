cmake_minimum_required(VERSION 3.0.2)
project(hill_start_assist)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()
catkin_package()

# 入口：main.py（--gif 才需要渲染依赖）
catkin_install_python(PROGRAMS main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
