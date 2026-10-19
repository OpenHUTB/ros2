# AirSim 无人机智能控制（神经网络感知·规划·控制）

本项目在 **AirSim 1.8.1**（亦可平滑迁移至 OpenHUTB 2.10.0 / CARLA 0.9.16）仿真环境下，
围绕无人机完成四项任务，所有感知、规划、控制与端到端策略均采用**神经网络**实现，
并在 **Ubuntu 20.04 + ROS Noetic / ROS2 Humble** 下通过 launch 一键启动。

> 仿真器负责渲染与传感器数据；本仓库负责控制算法、神经网络与文档。
> 每个模块入口均为 `main.py`（见 `modules/*/main.py`），并由 `scripts/main.sh` / `main.bat` 统一调度。

## 任务示例（点击进入）

| 任务 | 分值 | 一句话说明 | 示例文档 |
| --- | --- | --- | --- |
| ① 键盘运动控制 | 5 | 按键实时下发速度/偏航指令 | [任务1 →](tasks/task1_keyboard.md) |
| ② 传感器感知 + 轨迹控制 | 5×2 | 深度相机神经网络避障；神经网络轨迹跟踪 | [任务2 →](tasks/task2_perception.md) |
| ③ 建图与导航（SLAM 同步） | 5×3 | LiDAR/深度驱动探索，rtabmap 同步建图，move_base 导航 | [任务3 →](tasks/task3_slam_nav.md) |
| ④ 端到端模型 | 5 | 深度图像 → 速度指令的 CNN（+LSTM） | [任务4 →](tasks/task4_e2e.md) |

## 快速开始

```bash
git clone https://github.com/thechaos16/airsim_controller.git
cd airsim_controller
pip install -r requirements.txt

# 启动 AirSim 仿真器（Blocks 地图）后：
./scripts/main.sh keyboard      # 任务1
./scripts/main.sh avoid         # 任务2 避障
./scripts/main.sh track         # 任务2 轨迹跟踪
./scripts/main.sh explore       # 任务3 边飞边建图
./scripts/main.sh collect       # 任务4 采集
./scripts/main.sh train         # 任务4 训练
./scripts/main.sh e2e           # 任务4 端到端推理
```

ROS 启动：

```bash
roslaunch airsim_controller task1_keyboard.launch
roslaunch airsim_controller task2_avoid.launch
roslaunch airsim_controller task2_track.launch
roslaunch airsim_controller task3_slam_nav.launch
roslaunch airsim_controller task4_e2e.launch
```



## 目录导航

- 环境与安装：[Ubuntu+Noetic](getting-started/ubuntu_ros.md) ｜ [Humble](getting-started/ros2_humble.md) ｜ [AirSim](getting-started/airsim.md) ｜ [Docker/虚拟机/Windows](getting-started/docker_windows.md)
- [性能评价](performance.md)
