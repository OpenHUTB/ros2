cmake_minimum_required(VERSION 3.0.2)
project(mujoco_2link_impedance_control)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package(CATKIN_DEPENDS rospy)

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/output_paths.py
  scripts/impedance_controller.py
  scripts/mujoco_arm_impedance_node.py
  scripts/headless_test.py
  scripts/render_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
