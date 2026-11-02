cmake_minimum_required(VERSION 3.0.2)
project(mujoco_ground_slam_nn_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/nodes/sim_node.py
  scripts/nodes/slam_node.py
  scripts/nodes/planner_node.py
  scripts/nodes/controller_node.py
  scripts/nodes/perception_node.py
  scripts/nodes/e2e_node.py
  scripts/nodes/mission_node.py
  scripts/nodes/viewer_node.py
  scripts/nodes/teleop_keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY gvnn
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
        PATTERN "__pycache__" EXCLUDE)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        OPTIONAL)

