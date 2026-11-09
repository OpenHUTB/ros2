cmake_minimum_required(VERSION 3.0.2)
project(uav_altitude_sensor_fusion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(CATKIN_DEPENDS rospy std_msgs)

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/altitude_filter.py
  scripts/altitude_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
