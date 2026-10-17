cmake_minimum_required(VERSION 3.0.2)
project(carla_tsr_ros)

## Carla 交通标志识别 ROS 封装
##
## 包含：图像发布节点、YOLOv8 感知节点、控制节点
## 神经网络：YOLOv8n（ultralytics），检测 COCO 类别 11（stop sign）

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs geometry_msgs
)

#############
## Install ##
#############

## Python 节点
catkin_install_python(PROGRAMS
  scripts/image_publisher.py
  scripts/perception_node.py
  scripts/control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 启动文件、参数配置
install(DIRECTORY launch config assets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)

#############
## Testing ##
#############
## 测试规则后续补充
