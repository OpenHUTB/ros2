# carla_nn_perception_control_slam

基于神经网络的 CARLA 无人车**感知 / 规划 / 控制 / 端到端**仿真模块。

## 功能

| 任务 | 内容 | 神经网络 |
|------|------|----------|
| 1 | 物理仿真 + 键盘运动控制 | — |
| 2 | 传感器感知 + 给定轨迹控制 | YOLOv5m 目标检测 |
| 3 | 建图 + 导航（SLAM 同步仿真） | — |
| 4 | 端到端模型（图像 → 控制） | PilotNet（CNN） |

## 运行环境

| 项 | 要求 |
|----|------|
| CARLA | 0.9.13 |
| ROS | ROS2 Humble / Foxy（同时兼容 ROS1 Noetic 直接以 python3 运行） |
| Python | 3.8+ |
| 依赖 | 见 `requirements.txt` |

```bash
pip3 install -r requirements.txt
```

## 编译与运行

```bash
# 1. 把本目录放到工作空间
cd ~/ros2_ws/src && cp -r carla_nn_perception_control_slam . && cd ..

# 2. 编译
colcon build --packages-select carla_nn_perception_control_slam
source install/setup.bash

# 3. 启动 CARLA 服务端（另一终端，需 GPU）
# Windows 宿主机：
CarlaUE4.exe -quality-level=Low -windowed
# Linux：
./CarlaUE4.sh -quality-level=Low

# 4. 运行单个任务
ros2 launch carla_nn_perception_control_slam task1.launch.py carla_host:=127.0.0.1
ros2 launch carla_nn_perception_control_slam task2.launch.py
ros2 launch carla_nn_perception_control_slam task3.launch.py
ros2 launch carla_nn_perception_control_slam task4.launch.py

# 5. 或直接用入口 main
ros2 run carla_nn_perception_control_slam main --task 1
ros2 run carla_nn_perception_control_slam main --task 4 --mode drive --frames 70
```

任务4 完整训练流程（采集 → 训练 → 推理）：

```bash
ros2 run carla_nn_perception_control_slam main --task 4 --mode collect --frames 1200
ros2 run carla_nn_perception_control_slam main --task 4 --mode train --epochs 18
ros2 run carla_nn_perception_control_slam main --task 4 --mode drive --frames 70
```

## 权重文件

训练好的 PilotNet 权重（`models/pilotnet.pt`，约 1.6MB）已随仓库提交，任务 4 可直接进入 `drive` 推理模式。YOLOv5m 权重约 50MB，不随仓库提交，运行前放到：

```
~/models/yolov5m.pt          # 任务2 检测
~/e2e/pilotnet.pt            # 任务4 端到端（由 train 步骤产出）
```

下载地址：`https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov5mu.pt`（重命名 `yolov5m.pt`）。

## 输出

| 产物 | 说明 |
|------|------|
| `~/task1_manual_control.gif` | 任务1 演示 |
| `~/task2_perception_tracking.gif` + `task2_metrics.json` | 任务2 演示与指标 |
| `~/task3_slam_map.png` + `task3_slam_navigation.gif` + `task3_metrics.json` | 任务3 建图与导航 |
| `~/task4_e2e_pilotnet.gif` + `task4_metrics.json` + `~/e2e/train_curve.png` | 任务4 训练曲线与推理演示 |

## 目录结构

```
carla_nn_perception_control_slam/
├── package.xml
├── setup.py / setup.cfg
├── requirements.txt
├── resource/carla_nn_perception_control_slam
├── launch/            task1..task4.launch.py, all.launch.py
└── carla_nn_perception_control_slam/
    ├── main.py              # 统一入口
    ├── manual_control.py    # 任务1
    ├── perception_control.py# 任务2
    ├── slam_nav.py          # 任务3
    └── e2e_pilotnet.py      # 任务4
```

## 声明

本模块的代码与文档在编写过程中使用了大语言模型辅助，包括代码结构建议、公式整理与文字润色。

所有功能均已在 CARLA 0.9.13 + ROS 2 Humble / ROS 1 Noetic（Ubuntu 20.04）环境中实际运行验证，文中全部数值来自实测记录，没有估算或占位数据。作者已逐项核对提交内容，并对其负全部责任。
