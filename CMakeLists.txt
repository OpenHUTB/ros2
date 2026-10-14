cmake_minimum_required(VERSION 3.0.2)
project(mecanum_8shape_trajectory)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs geometry_msgs urdf xacro
  gazebo_ros robot_state_publisher controller_manager
  velocity_controllers joint_state_controller gazebo_msgs
  nav_msgs tf2 tf2_ros
)
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
)
include_directories(${catkin_INCLUDE_DIRS})
catkin_install_python(PROGRAMS
  scripts/mecanum_ctrl.py
  scripts/mecanum_kinematics.py
  scripts/trajectory_generator.py
  scripts/pid_tracker.py
  scripts/odom_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
