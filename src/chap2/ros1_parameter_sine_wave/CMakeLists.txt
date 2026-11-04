cmake_minimum_required(VERSION 3.0.2)
project(ros1_parameter_sine_wave)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  rospy
  std_msgs
)

catkin_python_setup()
generate_dynamic_reconfigure_options(cfg/Params.cfg)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure rospy std_msgs
)

catkin_install_python(
  PROGRAMS scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()

install(
  DIRECTORY cfg launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
