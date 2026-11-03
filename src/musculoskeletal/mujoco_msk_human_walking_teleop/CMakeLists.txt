cmake_minimum_required(VERSION 3.0.2)
project(mujoco_msk_human_walking_teleop)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  main.py
  scripts/msk_human_sim_node.py
  scripts/msk_human_keyboard_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 库模块（msk_human_core 等）安装到 share/<pkg>/scripts，供安装空间下
# roslib.packages.get_pkg_dir 定位导入；可执行节点只由上面的
# catkin_install_python 安装到 bin，此处排除避免重复安装
file(GLOB _LIB_SCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
list(FILTER _LIB_SCRIPTS EXCLUDE REGEX
  "/(main|msk_human_sim_node|msk_human_keyboard_node)\\.py$")
install(FILES ${_LIB_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
install(DIRECTORY launch models config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
