cmake_minimum_required(VERSION 3.0.2)
project(mujoco_vehicle_aeb)

find_package(catkin REQUIRED)

catkin_package()

# 入口脚本：包根 main.py（作业约定的入口）与 scripts/main.py（rosrun 兼容）
catkin_install_python(PROGRAMS
  main.py
  scripts/main.py
  scripts/cli.py
  scripts/aeb_controller.py
  scripts/aeb_env.py
  scripts/aeb_metrics.py
  scripts/runner.py
  scripts/plotting.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 模型、launch、说明
install(DIRECTORY models launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md requirements.txt package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/main.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
