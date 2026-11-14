cmake_minimum_required(VERSION 3.0.2)
project(carla_speed_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()

# 包根 main.py 是源码树里 `python3 main.py` 的入口，转调 scripts/main.py。
# 安装时只装 scripts/main.py（原名安装到 bin），两者同名，装包根版本会冲突，
# 所以这里保持只装一份；rosrun 与直接运行源码树的入口行为一致。
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
