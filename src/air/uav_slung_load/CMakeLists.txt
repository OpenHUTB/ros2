cmake_minimum_required(VERSION 3.0.2)
project(uav_slung_load)

find_package(catkin REQUIRED)
catkin_package()

# 入口与被 import 的模块必须装到同一目录，否则 rosrun 时 import 失败
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/evaluate.py
  scripts/sim_core.py
  scripts/controllers.py
  scripts/make_gif.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_offline.py)
endif()
