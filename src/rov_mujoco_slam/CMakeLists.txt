cmake_minimum_required(VERSION 3.8)
project(rov_mujoco_slam)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

# 安装 launch 文件夹、config 配置、models 模型
install(DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Python 脚本并赋予可执行权限
install(PROGRAMS
  scripts/main_sim_node.py
  scripts/sonar_slam_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()