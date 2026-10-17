cmake_minimum_required(VERSION 3.0.2)
project(carla_speed_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()

# 算法与节点脚本
catkin_install_python(PROGRAMS
  scripts/speed_control.py
  scripts/simple_plot.py
  scripts/speed_controller.py
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 launch 与参数文件
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()
