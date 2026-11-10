cmake_minimum_required(VERSION 3.0.2)
project(humanoid_carry_payload_nn)

find_package(catkin REQUIRED)

catkin_package()

# Python 入口与模块（catkin_install_python 会改写 shebang 并安装到 lib/<pkg>/）
# 三个被 import 的模块必须和入口脚本装到同一目录，否则 rosrun 时 import 会失败
catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/evaluate.py
  scripts/train_mlp.py
  scripts/make_gif.py
  scripts/gait.py
  scripts/model.py
  scripts/sim_core.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 模型 / launch / 权重
install(DIRECTORY models launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_offline.py)
endif()
