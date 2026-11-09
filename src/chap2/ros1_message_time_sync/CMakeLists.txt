cmake_minimum_required(VERSION 3.0.2)
project(ros1_message_time_sync)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS message_filters rospy sensor_msgs std_msgs
)

catkin_install_python(
  PROGRAMS scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_sensor_math.py)
endif()

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
