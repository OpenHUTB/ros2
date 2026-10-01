# my_car 差速轮小车URDF模型

## 简介
基于ROS Noetic的差速驱动小车URDF模型，支持RViz可视化显示与运动仿真。

## 环境依赖
- Ubuntu 20.04
- ROS Noetic
- robot_state_publisher
- rviz

## 文件结构
my_car/
├── urdf/
│ └── car.urdf # 小车 URDF 模型文件
├── config/
│ └── car.rviz # RViz 可视化配置文件
├── launch/
│ └── display.launch # 可视化启动文件
├── package.xml # ROS 功能包描述
└── README.md
plaintext

## 运行步骤
1. 将功能包放入ROS工作空间的src目录
2. 编译工作空间
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
启动可视化节点
bash
roslaunch my_car display.launch
