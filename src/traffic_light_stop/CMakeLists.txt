cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_stop)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/traffic_light_sim.py
  scripts/vehicle_model.py
  scripts/traffic_light.py
  scripts/stop_controller.py
  scripts/headless_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
