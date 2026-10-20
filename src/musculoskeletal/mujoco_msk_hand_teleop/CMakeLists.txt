cmake_minimum_required(VERSION 3.0.2)
project(mujoco_msk_hand_teleop)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  main.py
  scripts/msk_hand_core.py
  scripts/msk_hand_keyboard.py
  scripts/msk_hand_nn.py
  scripts/msk_hand_sim_node.py
  scripts/msk_hand_keyboard_node.py
  scripts/msk_hand_nn_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch models assets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
