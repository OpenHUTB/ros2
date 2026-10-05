cmake_minimum_required(VERSION 3.0.2)
project(airsim_controller)
find_package(catkin REQUIRED rospy std_msgs sensor_msgs nav_msgs geometry_msgs tf2_ros)
catkin_python_setup()
catkin_package()

# 安装 launch 文件与 ROS 节点包装脚本
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
catkin_install_python(PROGRAMS
  ros_nodes/task1_keyboard_main.py
  ros_nodes/task2_perception_main.py
  ros_nodes/task3_slam_main.py
  ros_nodes/task4_e2e_main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
