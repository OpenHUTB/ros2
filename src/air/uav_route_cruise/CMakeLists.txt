cmake_minimum_required(VERSION 3.0.2)
project(uav_route_cruise)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package()

# 入口脚本：用 catkin_install_python 安装（可执行，会包一层 wrapper）
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/train.py
  scripts/eval.py
  scripts/compare.py
  scripts/multiseed_eval.py
  scripts/make_gif.py
  scripts/route_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 被 import 的模块：必须以【普通文件】安装。
# 若也用 catkin_install_python，catkin 会给每个脚本包 wrapper
# （内容是 exec(compile(...))），于是 `import route_node` 拿到的是 wrapper，
# 其命名空间里没有 main 等函数，会报 AttributeError。
install(FILES
  scripts/policy.py
  scripts/route.py
  scripts/route_env.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
