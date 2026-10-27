# airsim_controller

基于 **AirSim 1.8.1**（可迁移 OpenHUTB 2.10.0 / CARLA 0.9.16）的无人机智能控制项目，
所有感知、规划、控制与端到端策略均采用**神经网络**实现，支持 **Ubuntu 20.04 + ROS Noetic / ROS2 Humble**，
每个模块以 `main.py` 为入口并支持 launch 一键启动。

在线文档（mkdocs + GitHub Pages）：见 `docs/`，构建后首页提供跳转各任务示例的链接。

## 任务清单

| 任务 | 说明 | 入口 | launch |
| --- | --- | --- | --- |
| ① 键盘控制 | 按键实时速度/偏航遥控 | `modules/task1_keyboard/main.py` | `launch/task1_keyboard.launch` |
| ② 感知+轨迹 | 深度相机神经网络避障；神经网络轨迹跟踪 | `modules/task2_perception_control/main.py` | `task2_avoid/track.launch` |
| ③ SLAM+导航 | LiDAR/深度驱动探索，rtabmap 同步建图，导航到点 | `modules/task3_slam_nav/main.py` | `launch/task3_slam_nav.launch` |
| ④ 端到端 | 深度图像 → 速度指令（CNN/CNN-LSTM），含采集/训练/推理 | `modules/task4_e2e/main.py` | `launch/task4_e2e.launch` |

## 运行环境

- Ubuntu 20.04（推荐）/ 22.04 / Windows 原生；AirSim 1.8.1；ROS Noetic 或 Humble（任务3）。
- Python 依赖：`pip install -r requirements.txt`（airsim、numpy、opencv、pynput、torch、mkdocs-material）。

## 快速运行

```bash
# 1. 启动 AirSim 仿真器
# 2. Linux
./scripts/main.sh keyboard
./scripts/main.sh avoid
./scripts/main.sh track
./scripts/main.sh explore
./scripts/main.sh navigate 5 5
./scripts/main.sh collect && ./scripts/main.sh train && ./scripts/main.sh e2e

# Windows
scripts\main.bat keyboard

# ROS
roslaunch airsim_controller task1_keyboard.launch
```

## 目录结构

```
airsim_controller/
├── common/            # 扩展 DroneClient + 神经网络模型库
├── modules/           # 四个任务，每个含 main.py 入口
├── comparison/         # PPO/DQN/SAC 与 MLP/CNN/CNN-LSTM 对比
├── launch/            # ROS launch 文件
├── ros_nodes/         # ROS 节点包装脚本
├── scripts/           # main.sh / main.bat 统一入口
├── docs/              # mkdocs 文档站（mkdocs.yml + docs/）
├── Dockerfile
├── package.xml / CMakeLists.txt   # catkin 包
└── requirements.txt
```

## 文档构建与发布

```bash
cd docs
pip install mkdocs mkdocs-material
mkdocs serve          # 本地预览 http://127.0.0.1:8000
mkdocs build           # 生成 site/
mkdocs gh-deploy      # 发布到 GitHub Pages
```
