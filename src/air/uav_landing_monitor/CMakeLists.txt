cmake_minimum_required(VERSION 3.0.2)
project(uav_landing_monitor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  std_msgs
)

catkin_package()

# 只把真正的入口脚本当可执行程序安装；
# monitor_core.py 是库（被节点 import），tools/plot_report.py 依赖 matplotlib，
# 都不适合走 catkin_install_python，改为随 share 目录一起安装。
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/landing_monitor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config scripts tools
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

# 纯 python 回归测试（不依赖 ROS master / 仿真器），catkin_make run_tests 会一起跑。
# nose 只收集 test 开头的函数，场景函数都叫 scenario_*，所以文件里另外暴露了
# test_all_scenarios() 作为收集入口（本地直跑仍会打印逐条结果）。
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/test_uav_landing_monitor_local.py)
endif()
