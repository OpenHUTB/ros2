# hutb_pilotnet_e2e

实现**端到端模型**：输入前视单目相机图像（160×96×3），经 **PilotNet 风格卷积神经网络**
直接输出控制指令（转向角 steer、速度 speed），映射为 `/cmd_vel` 驱动 TurtleBot3，
展示"**图像 → 控制指令**"的端到端学习范式。

## 运行环境

- Ubuntu 20.04 + ROS Noetic + Gazebo 11
- Python 3.8 + NumPy + PyTorch + OpenCV，依赖 `turtlebot3_description` / `gazebo_ros` / `cv_bridge`

## 运行步骤

```bash
# 1) 编译安装
cd ~/catkin_ws/src && git clone <本模块> hutb_pilotnet_e2e
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# 2) 数据采集（自动落盘 data/training/*.npy；驾驶方式任选：teleop 键盘或任务2 模块漫游）
roslaunch hutb_pilotnet_e2e main.launch mode:=collect

# 3) 训练（30 epoch，输出 models/pilotnet.pt）
python3 $(rospack find hutb_pilotnet_e2e)/scripts/train_end2end.py

# 4) 实时推理（图像 -> CNN -> /cmd_vel 闭环）
roslaunch hutb_pilotnet_e2e main.launch mode:=inference
```

## 端到端链路

```
前视相机 (160x96 RGB) -> 归一化 [-1,1] -> PilotNet CNN -> (steer, speed)
-> /cmd_vel (linear.x=speed, angular.z=steer) -> 差速驱动 -> 相机新帧（闭环）
```

## 网络结构

```
Input (3x160x96) -> Conv5x5x24/s2 -> Conv5x5x36/s2 -> Conv5x5x48/s2
-> Conv3x3x64/s1 -> Conv3x3x64/s1 -> Dropout(0.5) -> Flatten(4160)
-> FC100 -> FC50 -> FC10 -> FC2
```

## 目录结构

```
launch/main.launch          mode:=inference|collect 一键启动
scripts/main.py             实时推理节点
scripts/data_collector.py   数据采集节点
scripts/train_end2end.py    训练脚本
models/pilotnet.py          PilotNet 网络定义
models/pilotnet.pt          预训练权重
worlds/task3_world.world    仿真场景
config/e2e.yaml             参数配置
```
