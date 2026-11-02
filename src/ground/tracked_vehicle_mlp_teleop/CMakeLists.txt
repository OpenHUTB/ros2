cmake_minimum_required(VERSION 3.0.2)
project(tracked_vehicle_mlp_teleop)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
  gazebo_ros
  gazebo_plugins
  xacro
  robot_state_publisher
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    std_srvs
    tf
    tf2_ros
    gazebo_ros
    gazebo_plugins
    xacro
    robot_state_publisher
)

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/evaluate.py
  scripts/train_mlp.py
  scripts/model.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # 离线 pytest 用例（不依赖 ROS/Gazebo），catkin_make run_tests 会自动执行
  catkin_add_nosetests(scripts/test_offline.py TIMEOUT 120)
endif()

install(DIRECTORY launch urdf worlds config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
