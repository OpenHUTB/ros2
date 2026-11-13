cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_teleop)

# 查找 catkin 及本模块用到的依赖：rospy 负责节点与话题，geometry_msgs 提供 Twist 消息，
# turtlesim 提供仿真器节点。
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  turtlesim
)

catkin_package()

# 把键盘遥控入口脚本 main.py 安装到 bin 目录，装好后可用
# rosrun turtlesim_teleop main.py 直接运行。
# 注意：clone 后需保证脚本有可执行权限（chmod +x main.py）。
catkin_install_python(PROGRAMS
  main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 launch 目录到 share/turtlesim_teleop，保证 catkin_make install 或
# 发布场景下 roslaunch turtlesim_teleop main.launch 能找到文件。
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
