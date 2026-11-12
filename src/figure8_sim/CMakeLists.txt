cmake_minimum_required(VERSION 3.0.2)
project(figure8_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# pure-python helper module: installed next to main.py so main.py can import it
install(PROGRAMS
  scripts/figure8.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
