cmake_minimum_required(VERSION 3.0.2)
project(autonomous_car)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_package()

# 告诉 catkin 把 main.py 安装为可执行程序（因为你的 main.py 直接放在根目录下）
catkin_install_python(PROGRAMS main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)