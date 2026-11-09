cmake_minimum_required(VERSION 3.0.2)
project(cartpole_balance)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

catkin_install_python(PROGRAMS
  scripts/balance_controller.py
  scripts/cartpole_sim.py
  scripts/cartpole_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
