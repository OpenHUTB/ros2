cmake_minimum_required(VERSION 2.8.3)
project(turtle_sim_experiment)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  turtlesim
)

catkin_package()

# 安装 Python 入口脚本 main.py。
# 注意：clone 后需保证脚本有可执行权限（chmod +x main.py），
# 脚本采用 Python 2/3 兼容写法：Noetic（Python 3）与 Kinetic（Python 2.7）均可运行。
catkin_install_python(PROGRAMS
  main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
