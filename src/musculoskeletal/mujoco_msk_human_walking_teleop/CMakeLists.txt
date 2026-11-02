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
# roslib.packages.get_pkg_dir 定位导入；models/config 同理供参数与模型加载
install(DIRECTORY launch models config scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
