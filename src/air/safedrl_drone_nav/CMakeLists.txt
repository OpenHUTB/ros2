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
# files, configs, model and checkpoints into the catkin install space.
catkin_install_python(PROGRAMS
  scripts/safe_nav_env.py
  scripts/nav_config.py
  scripts/safety.py
  scripts/sim_frames.py
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
  scripts/airsim_bridge_node.py
  scripts/mujoco_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

# Trained weights - keep them in the package so a reviewer can immediately run
# the inference node without training anything.
install(DIRECTORY checkpoints/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/checkpoints
  OPTIONAL
)

# Unit tests are plain Python and need no ROS master:
#   python tests/test_safety_filter.py
#   python tests/test_bridge_frames.py
# The closed-loop test also needs the offline PyBullet environment:
#   python tests/test_environment.py
