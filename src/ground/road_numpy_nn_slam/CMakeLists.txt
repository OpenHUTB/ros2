cmake_minimum_required(VERSION 3.0.2)
project(road_numpy_nn_slam)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/sim_node.py
  scripts/slam_node.py
  scripts/nav_node.py
  scripts/perception_node.py
  scripts/e2e_node.py
  scripts/metrics_node.py
  scripts/viewer_node.py
  scripts/teleop_keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY road_numpy_nn_slam
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
        PATTERN "__pycache__" EXCLUDE)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        OPTIONAL)
install(DIRECTORY models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        OPTIONAL)
install(DIRECTORY docs
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        OPTIONAL)
install(PROGRAMS
        scripts/build_ros.sh
        scripts/run_demo.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        OPTIONAL)
