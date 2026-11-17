# 环境总览

本项目支持以下运行环境（推荐 **Ubuntu 20.04 + ROS Noetic**）：

| 环境 | 仿真器 | ROS | 适用任务 |
| --- | --- | --- | --- |
| Ubuntu 20.04 原生 | AirSim 1.8.1 / OpenHUTB | Noetic (Python3) | 全部任务 |
| Ubuntu 22.04 | AirSim / OpenHUTB | Humble | 全部任务（含 AirSim ROS2 示例） |
| Windows 10/11 原生 | AirSim 1.8.1 | 无（纯 Python API） | 任务1/2/4 |
| Docker（osrf/ros:noetic） | 仿真器在宿主机 | Noetic | 全部（网络连接宿主机） |
| VMware/VirtualBox 虚拟机 | 仿真器在宿主机 | Noetic/Humble | 全部 |

## 硬件要求

- CPU 4 核以上、内存 16 GB 以上；
- 独立显卡（AirSim 为 Unreal Engine 渲染，建议 GTX 1060 以上）；
- 磁盘 30 GB 以上（Unreal + AirSim 可执行文件约 10 GB）。

## 安装顺序

1. [安装 AirSim 仿真器](airsim.md)并跑通默认 Blocks 地图；
2. [安装 ROS Noetic 或 Humble](ubuntu_ros.md)；
3. 克隆本仓库并 `pip install -r requirements.txt`；
4. 按[任务页面](../tasks/task1_keyboard.md)逐个验证。

## 验证 API 连通性

启动 AirSim 后，另开终端执行：

```bash
python -c "import airsim; c=airsim.MultirotorClient(); c.confirmConnection(); print(c.getMultirotorState().kinematics_estimated.position)"
```

能打印出位置坐标即说明控制通道打通。
