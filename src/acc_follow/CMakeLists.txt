cmake_minimum_required(VERSION 3.0.2)
project(acc_follow)

find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs std_msgs)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/acc_sim.py
  scripts/headless_test.py
  scripts/vehicle_model.py
  scripts/acc_controller.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
