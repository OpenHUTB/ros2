cmake_minimum_required(VERSION 3.0.2)
project(smartcar_description)

find_package(catkin REQUIRED COMPONENTS
  xacro
)

catkin_package()

# 安装 xacro 模型
install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装启动文件
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装配置文件（控制器 yaml、rviz 配置）
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
