# RVNN：MuJoCo 6-DOF 水下机器人物理仿真、声呐 SLAM 与纯 NumPy 神经网络导航

本模块是 `OpenHUTB/ros2` 仓库的**水下机器人（ROV）**示例，对应课程作业中「水下机器人」选题的完整实现：

1. **物理仿真 + 键盘运动控制**：MuJoCo 3.2.3 构建 6-DOF ROV 刚体模型，显式实现重力/浮力/附加质量/二次流体阻力/六推进器推力分配，`W/A/S/D/R/F/Q/E` 键盘遥控；
2. **传感器感知与运动控制**：64 波束 120° 成像声呐（含噪声、野值、丢包）、DVL、IMU、深度计、前视相机；感知 MLP 对声呐去噪后再送入控制器；
3. **建图与导航**：对数几率占据栅格地图 + 相关性扫描匹配 SLAM、A* 全局规划、DWA 局部规划、纯跟踪 + 深度 PID 控制，导航与 SLAM 同步运行；
4. **端到端模型**：相机图像（48×64×3）→ 2 层 CNN → 速度指令 `(vx, wz)`，行为克隆训练；
5. **神经网络训练 / 测试 / ROS 封装**：纯 NumPy 实现 MLP/CNN/Adam/DQN/REINFORCE，7 个 `.npz` 模型，8 个 ROS 节点、6 个 launch；
6. **多场景性能评价**：`pool` / `harbor` / `cave` 三种水下场景闭环 benchmark 与 RL 算法/网络结构对比。

完整的原理推导、算法流程、源码解析与实验数据见文档页：[`docs/water/rov_numpy_nn_slam.md`](../../../docs/water/rov_numpy_nn_slam.md)。

## 目录结构

```
rov_numpy_nn_slam/
├── main.sh                  # 统一入口（teleop / slam_nav / perception / e2e / benchmark / test / train / setup）
├── rov_numpy_nn_slam/       # 核心 Python 包（纯 NumPy + MuJoCo）
│   ├── config.py            # 常量（地图分辨率、传感器量程、限速、场景）
│   ├── scenes.py            # pool / harbor / cave 场景与 MJCF 模型生成
│   ├── sim.py               # 6-DOF 动力学 + 声呐/DVL/IMU/深度计/相机
│   ├── render.py            # 俯视图渲染（仿真 / 地图 / 路径 / HUD）
│   ├── slam.py              # 占据栅格 + 扫描匹配 SLAM
│   ├── planning.py          # A* 全局规划 + DWA 局部规划
│   ├── control.py           # 深度 PID、纯跟踪、安全滤波、任务状态机
│   ├── datasets.py          # 专家数据采集与特征构造
│   ├── runtime.py           # roslaunch（系统 Python 3.8）自动切换到项目 venv（Python 3.11）
│   └── nn/                  # MLP / CNN / Adam / DQN / REINFORCE
├── scripts/                 # ROS 节点（sim/slam/nav/perception/e2e/metrics/viewer/teleop）
├── launch/                  # teleop / slam_nav / perception / e2e / benchmark / bringup
├── tools/                   # 采集、训练、评测、GIF 录制
├── models/                  # 训练好的 .npz 模型（1.9 MB）
├── results/                 # 训练指标、benchmark、ROS 运行记录（JSON）
└── tests/                   # 物理 / 建图 / 规划 / 控制 自测
```

## 环境要求

- Ubuntu 20.04.6 LTS + ROS Noetic（`/opt/ros/noetic`）
- Python ≥ 3.10（MuJoCo 3.2.3 要求；Ubuntu 20.04 自带 3.8，因此项目使用私有 venv）
- 依赖：`mujoco==3.2.3`、`numpy==1.26.4`、`Pillow`、`PyYAML`、`rospkg`（见 `requirements.txt`）
- 无 GPU、无网络环境可完整运行（训练与评测均只用 CPU）

## 快速开始

```bash
# 0) 进入模块目录
cd src/water/rov_numpy_nn_slam

# 1) 创建项目私有 Python 环境（不修改系统 Python / ~/.bashrc）
bash main.sh setup          # 等价于 bash scripts/setup_env.sh

# 2) 编译独立 catkin 工作空间（默认 ~/rov_numpy_nn_slam_ws，不覆盖已有工作空间）
bash scripts/build_ros.sh

# 3) 键盘遥控（物理仿真 + 查看器）
source ~/rov_numpy_nn_slam_ws/devel/setup.bash
roslaunch rov_numpy_nn_slam teleop.launch scene:=pool
# 或使用统一入口：bash main.sh teleop pool
```

键盘（先点击 “RVNN ROV” 窗口使其获得焦点）：

| 按键 | 作用 |
| --- | --- |
| `W` / `S` | 前进 / 后退（surge） |
| `A` / `D` | 左转 / 右转（yaw） |
| `R` / `F` | 上浮 / 下潜（heave） |
| `Q` / `E` | 左移 / 右移（sway） |
| `Space` | 停止 |

## 运行模式

| 命令 | 说明 |
| --- | --- |
| `bash main.sh teleop pool` | MuJoCo 物理仿真 + 键盘遥控 |
| `bash main.sh slam_nav harbor` | 声呐建图 + 扫描匹配 + A* + 纯跟踪自主导航 |
| `bash main.sh perception pool` | 感知 MLP 声呐去噪，发布 `/rov/scan_filtered` |
| `bash main.sh e2e cave` | 端到端 CNN：相机图像 → 速度指令 |
| `bash main.sh benchmark pool` | 指标评测（写 `results/ros_*.json`） |
| `bash main.sh test` | 物理 / 建图 / 规划 / 控制 / 网络自测 |
| `bash main.sh train` | 重新采集数据并训练全部网络（CPU 约 10 分钟） |

## 实测结果（Ubuntu 20.04 虚拟机，4 核 CPU，无 GPU）

| 指标 | 结果 |
| --- | --- |
| 经典 SLAM + A* + 纯跟踪（三场景） | **成功率 100%**（pool 13.6 s / harbor 16.4 s / cave 21.9 s，0 碰撞，终点误差 ≈0.70 m） |
| SLAM 绝对轨迹误差 ATE RMSE | **0.183 m**（地图覆盖率 61.9%） |
| 声呐去噪（感知 MLP） | RMSE **1.092 → 0.632 m（−42.2%）** |
| 神经控制（行为克隆） | vx MAE **0.0169 m/s**，yaw MAE **0.0551 rad/s** |
| 神经两级控制（规划+控制，pool） | 成功率 **100%**（14.7 s，0 碰撞） |
| 端到端 CNN | vx MAE **0.247 m/s**，闭环成功率 0%（见文档失败分析） |
| 强化学习对比（8 动作离散任务） | REINFORCE **96.7%** / DQN 2×64 **60.0%** / DQN 3×64 **26.7%** |
| ROS 话题速率（teleop） | `/rov/scan` 10 Hz、`/rov/state` 20 Hz、`/rov/odom` 50 Hz |

## 复现步骤

```bash
bash main.sh setup
bash main.sh test
bash scripts/build_ros.sh
bash main.sh slam_nav pool        # 观察查看器窗口中的地图、路径与 HUD
bash main.sh train                # 重新训练（约 10 分钟）
.venv/bin/python tools/evaluate.py --scenes pool,harbor,cave
```

## 已知限制

- 单级神经控制与端到端 CNN 缺少全局路径记忆，闭环成功率低（0%）；两级神经控制（`planner_mlp` → `control_mlp`）在 pool 达到 100%，harbor/cave 仍受 planner 训练标签以直行为主的限制。
- 仿真为 6-DOF 刚体 + 显式水动力模型，未接入真实水动力参数辨识。
- 声呐为射线仿真（非声学渲染），未建模多径与混响。

## 参考与说明

- 场景与 ROS 组织方式参考本仓库 `docs/water/rov_physical_simulation.md`、`docs/water/rov_sonar_slam_navigation.md` 等已有实现；
- 本模块所有代码、模型与数据均由本项目自行实现/生成，未使用他人代码；
- 物理引擎：MuJoCo 3.2.3；ROS：Noetic；神经网络：纯 NumPy（无 PyTorch/TensorFlow）。
