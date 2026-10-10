cmake_minimum_required(VERSION 3.0.2)
project(safedrl_drone_nav)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

# The package is pure Python, so "installing" is copying the scripts, launch
# files, configs, checkpoints and the evidence artefacts into the catkin
# install space.
catkin_install_python(PROGRAMS
  scripts/safe_nav_env.py
  scripts/safety.py
  scripts/drone_dynamics.py
  scripts/lagrangian_ppo.py
  scripts/run_logging.py
  scripts/figures.py
  scripts/train.py
  scripts/evaluate.py
  scripts/pretrain_bc.py
  scripts/make_report_figures.py
  scripts/drl_navigator_node.py
  scripts/dummy_state_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Trained weights - keep them in the package so a reviewer can immediately run
# the inference node without training anything.
install(DIRECTORY checkpoints/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/checkpoints
  OPTIONAL
)

# Evidence artefacts (logs + figures) are installed so that `roslaunch ...`
# users can point at them directly.
install(DIRECTORY docs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/docs
  OPTIONAL
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Safety-filter unit tests do not need a ROS master, but wiring them into
  # rostest makes `catkin_make run_tests` cover them.
  add_rostest(tests/safety_filter.test)
endif()
