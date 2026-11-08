cmake_minimum_required(VERSION 3.0.2)
project(mujoco_skid_steer_offroad_stable)

find_package(catkin REQUIRED)

catkin_package()

# 仿真主程序与姿态补偿控制器装到 lib/<包名>/，两者同目录保证 main.py 里的 import 成立
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/stable_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 模型、launch、依赖清单随包安装到 share/<包名>/
install(DIRECTORY models launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
