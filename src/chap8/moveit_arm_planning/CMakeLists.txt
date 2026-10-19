cmake_minimum_required(VERSION 3.0.2)
project(moveit_arm_planning)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_planners_ompl
  moveit_ros_move_group
  moveit_ros_planning_interface
  moveit_ros_visualization
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/plan_eval.py
  scripts/pick_place_task.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
