cmake_minimum_required(VERSION 3.0.2)
project(airsim_rl_planner_ros1)
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs sensor_msgs std_msgs std_srvs cv_bridge)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/bridge_node.py scripts/ppo_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
