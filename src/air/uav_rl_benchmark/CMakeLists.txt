cmake_minimum_required(VERSION 3.0.2)
project(uav_rl_benchmark)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

# 入口脚本：用 catkin_install_python 安装（可执行，会包一层 wrapper）
catkin_install_python(PROGRAMS
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 被 import 的模块：必须以【普通文件】安装。
# 若也用 catkin_install_python，catkin 会给脚本包 wrapper（exec(compile(...))），
# `import benchmark` 拿到的是 wrapper，命名空间里没有需要的函数。
install(FILES
  scripts/nav_env.py
  scripts/algos.py
  scripts/benchmark.py
  scripts/report.py
  scripts/benchmark_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
