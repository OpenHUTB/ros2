cmake_minimum_required(VERSION 3.0.2)
project(uav_camera_frustum_viz)
find_package(catkin REQUIRED COMPONENTS
  rospy geometry_msgs nav_msgs sensor_msgs visualization_msgs)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/frustum_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
