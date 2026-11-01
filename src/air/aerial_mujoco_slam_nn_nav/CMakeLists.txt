cmake_minimum_required(VERSION 3.0.2)
project(aerial_mujoco_slam_nn_nav)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs nav_msgs sensor_msgs tf)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS
  scripts/sim_node.py
  scripts/teleop_node.py
  scripts/nav_node.py
  scripts/e2e_node.py
  scripts/metrics_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()
