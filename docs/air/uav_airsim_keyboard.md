---
title: AirSim 四旋翼无人机键盘遥控
---

# AirSim 四旋翼无人机键盘遥控

基于 ROS2 Humble 与 AirSim（Blocks 环境）的无人机键盘遥控实验：在 AirSim 仿真中通过键盘手动控制四旋翼飞行，并实时发布位姿话题，用于验证 AirSim-RPC 连接、ROS2 话题发布与控制指令下发链路。

## 1. 功能概述

| 功能 | 说明 |
| --- | --- |
| 仿真环境 | AirSim Blocks（Windows 宿主机，API 端口 8080） |
| 通信方式 | 虚拟机通过 `10.0.2.2:8080` 连接 AirSim RPC |
| 控制方式 | 键盘遥控（速度指令 `moveByVelocityAsync`） |
| 发布话题 | `/uav/pose`（`geometry_msgs/PoseStamped`，AirSim NED 系） |
| 控制频率 | 10 Hz |

## 2. 目录结构

```
src/uav_airsim_keyboard/
├── package.xml                 # ROS2 包描述（ament_python）
├── setup.py / setup.cfg        # 安装配置
├── uav_airsim_keyboard/
│   └── main.py                 # 节点：AirSim 连接 + 键盘控制 + 位姿发布
└── launch/
    └── uav_airsim_keyboard.launch.py
```

## 3. 运行步骤

1. Windows 上启动 AirSim Blocks：`D:\airsim\Blocks\WindowsNoEditor\Blocks.exe`
2. 虚拟机中构建并启动：

```bash
cd ~/ros2
colcon build --packages-select uav_airsim_keyboard
source install/setup.bash
ros2 launch uav_airsim_keyboard uav_airsim_keyboard.launch.py
```

3. 按键说明：

| 按键 | 动作 |
| --- | --- |
| T | 起飞 |
| W / S | 前 / 后 |
| A / D | 左 / 右 |
| R / F | 上升 / 下降 |
| Q / E | 左转 / 右转 |
| L | 降落 |
| Ctrl+C | 退出（自动降落） |

## 4. 话题说明

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/uav/pose` | `geometry_msgs/PoseStamped` | 无人机位姿（AirSim NED 系） |

查看位姿：

```bash
ros2 topic echo /uav/pose
```

## 5. 运行效果

![AirSim 键盘遥控运行效果](../../img/uav_airsim_keyboard_demo.gif)
