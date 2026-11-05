cmake_minimum_required(VERSION 3.0.2)
project(smartcar_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 键盘遥控与阿克曼解算节点
add_executable(smartcar_control_gazebo src/smartcar_control_gazebo.cpp)
target_link_libraries(smartcar_control_gazebo ${catkin_LIBRARIES})

# 安装可执行文件
install(TARGETS smartcar_control_gazebo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装启动文件
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装脚本
catkin_install_python(PROGRAMS
  scripts/front_wheel_jsp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
