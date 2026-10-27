cmake_minimum_required(VERSION 3.0.2)
project(mujoco_msk_human_walking_teleop)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  main.py
  scripts/msk_human_core.py
  scripts/msk_human_keyboard.py
  scripts/msk_human_eval.py
  scripts/msk_human_sim_node.py
  scripts/msk_human_keyboard_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch models config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
