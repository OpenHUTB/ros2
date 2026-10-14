cmake_minimum_required(VERSION 3.0.2)
project(auto_park_hybrid_a_star)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/reeds_shepp.py
  scripts/hybrid_a_star.py
  scripts/collision_check.py
  scripts/tracker.py
  scripts/auto_park_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
