cmake_minimum_required(VERSION 3.0.2)
project(whole_body_ik)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/main.py
  main.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# main.py 用 sys.path 找同目录的模块，所以这些 .py 必须和它装到同一个
# 目录，否则 catkin_make install 之后会 ModuleNotFoundError。
install(FILES
  scripts/linalg.py
  scripts/kinematics.py
  scripts/ik_solver.py
  scripts/manipulability.py
  scripts/animate.py
  scripts/simple_plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
