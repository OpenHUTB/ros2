# 机器人的“黑匣子”——ROS1 Bag录包数据的离线挖掘与可视化
## 实验主题
机器人的“黑匣子”——ROS 1 Bag录包数据的离线挖掘与可视化

## 实验背景
常规仿真实验只关注机器人能否正常运动，仿真结束后传感器、控制数据直接丢失。
工程开发中可以使用 rosbag 录制全部话题数据，在仿真结束后离线复盘，分析控制误差、定位故障，相当于机器人的飞行黑匣子。

## 实验目标
1. 搭建图像录制ROS封装模块，订阅相机话题，保存图像；
2. 录制 rosbag 数据包，包含 /cmd_vel、/odom、/scan、图像话题；
3. 离线读取bag包，提取控制指令、里程计、激光雷达数据；
4. 可视化绘制：指令速度-实际速度曲线、速度误差曲线、激光雷达点云；
5. 分析机器人运动误差、卡顿、障碍物干扰现象。

## 代码位置
源码路径：`src/chap5/bag_blackbox`

## 目录结构
src/chap5/bag_blackbox/

├── bag_blackbox/

│   ├── __init__.py

│   ├── bag_analysis_node.py

│   └── image_record_node.py

├── launch/

│   └── bag_blackbox.launch.py

├── resource/

│   └── bag_blackbox

├── package.xml

├── setup.cfg

└── setup.py

## 环境
- Ubuntu 20.04 + ROS Noetic
- 依赖：rospy, rosbag, sensor_msgs, cv_bridge, opencv, matplotlib, numpy, scipy

## 实验步骤
1. 启动TurtleBot3 Gazebo仿真
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
## 效果图
