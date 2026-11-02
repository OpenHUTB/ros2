cmake_minimum_required(VERSION 3.0.2)
project(humanoid_carry_payload_nn)

find_package(catkin REQUIRED)

catkin_package()

# Python 入口（catkin_install_python 会改写 shebang 并安装到 lib/<pkg>/）
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/evaluate.py
  scripts/train_mlp.py
  scripts/make_gif.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 模型 / 配置 / launch / 权重
install(DIRECTORY models config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 供入口脚本 import 的模块（sim_core / gait / model）
install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_offline.py)
endif()
