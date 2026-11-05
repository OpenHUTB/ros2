cmake_minimum_required(VERSION 3.0.10)
project(carla_sensor_fusion)

find_package(catkin REQUIRED)

catkin_package()

# ---------------------------------------------------------------
# 安装 python 入口与依赖模块到 devel/install 空间，
# 使 roslaunch 的 type="main.py" 能被可靠解析，且 main.py 在同目录即可
# import 到 sensor_fusion_core / fusion_perception / sim_2d_fusion / carla_sensor_fusion。
# ---------------------------------------------------------------
catkin_install_python(PROGRAMS
  main.py
  carla_sensor_fusion.py
  sim_2d_fusion.py
  sensor_fusion_core.py
  fusion_perception.py
  test_sensor_fusion.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch 文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
