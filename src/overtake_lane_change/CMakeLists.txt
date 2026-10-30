cmake_minimum_required(VERSION 3.0.2)
project(overtake)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs
)

catkin_install_python(PROGRAMS
  scripts/vehicle_model.py
  scripts/overtake_controller.py
  scripts/headless_test.py
  scripts/overtake_sim.py
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
