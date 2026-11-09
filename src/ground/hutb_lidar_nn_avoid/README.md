# hutb_lidar_nn_avoid

TurtleBot3 搭载 2D 激光雷达（LDS-01），将 `/scan` 原始测距数据**降采样为 60 维特征向量**，
输入**多层感知机（MLP，60→64→32→2）神经网络**，网络直接输出（线速度 v，角速度 ω）控制指令，
实现**基于传感器感知的神经网络运动控制（避障）**。

## 运行环境

- Ubuntu 20.04 + ROS Noetic + Gazebo 11
- Python 3.8 + NumPy，依赖 `turtlebot3_description` / `turtlebot3_gazebo` / `gazebo_ros` / `xacro`

## 运行步骤

```bash
# 1) 编译安装
cd ~/catkin_ws/src && git clone <本模块> hutb_lidar_nn_avoid
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# 2) 一键启动：Gazebo 障碍世界 + TurtleBot3 + MLP 避障节点
roslaunch hutb_lidar_nn_avoid main.launch

# 3) 可选：重新训练（行为克隆 4000 样本，600 epoch）
rosrun hutb_lidar_nn_avoid main.py train
```

## 感知与控制链路

```
激光雷达 /scan (360 点) -> 60 等分取最小 + 归一化 -> MLP 前向 -> (v, w)
-> 限幅 v∈[0,0.22], w∈[-1.5,1.5] -> /cmd_vel -> 差速驱动插件
```

## 目录结构

```
launch/main.launch          一键启动
scripts/main.py             训练 / 实时避障节点（MLP 纯 NumPy 实现）
models/lidar_mlp.npz        预训练权重
worlds/avoid_world.world    自建障碍场景
config/nn.yaml              参数配置
```
