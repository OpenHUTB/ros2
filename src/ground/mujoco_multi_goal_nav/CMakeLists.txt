cmake_minimum_required(VERSION 3.10)
project(mujoco_multi_goal_nav)

find_package(catkin REQUIRED)

catkin_package()

# 入口脚本装到 lib/<pkg>/，供 roslaunch 以 type="main.py" 启动
catkin_install_python(PROGRAMS main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# MuJoCo 模型与脚本副本装到 share/<pkg>/，便于从 share 目录直接 python3 main.py 运行
install(FILES main.py scene.xml requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# launch 装到 share/<pkg>/launch/
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
