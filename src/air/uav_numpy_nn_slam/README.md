# UAVNN：MuJoCo 四旋翼无人机物理仿真、激光 SLAM 与纯 NumPy 神经网络导航

本模块是 `OpenHUTB/ros2` 仓库的**空域载具（四旋翼无人机）**示例，对应课程作业「无人机/空域载具」选题的完整实现：

1. **物理仿真 + 键盘运动控制**：MuJoCo 3.1–3.2（requirements 固定 `mujoco>=3.1,<3.3`） 真实重力/机体推力/姿态 PD/气动阻力/风扰的 6-DOF 四旋翼模型，`W/A/S/D/R/F/Q/E` 键盘遥控；
2. **传感器感知与运动控制**：72 波束 360° 激光雷达（含噪声/野值/丢点）、气压高度计、GPS、IMU、前视相机；感知 MLP 对激光去噪后再送入控制器；
3. **建图与导航**：对数几率占据栅格 + 相关性扫描匹配 SLAM、A* 全局规划（建筑 + 禁飞区硬约束）、DWA 局部规划、纯跟踪 + 高度 PID；
4. **端到端模型**：相机图像（48×64×3）→ 2 层 CNN → 速度指令 `(vx, yaw_rate)`，行为克隆训练；
5. **神经网络训练 / 测试 / ROS 封装**：纯 NumPy 实现 MLP/CNN/Adam/DQN/REINFORCE，7 个 `.npz` 模型、8 个 ROS 节点、6 个 launch；
6. **多场景性能评价**：`city`（城市）、`canyon`（峡谷）、`airport`（机场）三场景闭环 benchmark，包含禁飞区侵入次数与 RL 算法/网络结构对比。

完整原理、算法流程与实验数据见文档页：[`docs/air/uav_numpy_nn_slam.md`](../../../docs/air/uav_numpy_nn_slam.md)。

## 目录结构

```
uav_numpy_nn_slam/
├── main.sh                    # 统一入口（teleop / slam_nav / perception / e2e / benchmark / test / train / setup）
├── uav_numpy_nn_slam/         # 核心 Python 包（纯 NumPy + MuJoCo）
│   ├── config.py              # 常量（地图分辨率、雷达量程、风速、限速、场景）
│   ├── scenes.py              # city / canyon / airport 场景、禁飞区与 MJCF 生成
│   ├── sim.py                 # 6-DOF 四旋翼动力学 + 激光/气压/GPS/IMU/相机
│   ├── render.py              # 俯视图渲染（建筑/禁飞区/雷达/路径/HUD）
│   ├── slam.py                # 占据栅格 + 扫描匹配 SLAM
│   ├── planning.py            # A*（含禁飞区） + DWA 局部规划
│   ├── control.py             # 高度 PID、纯跟踪、安全滤波、任务状态机
│   ├── datasets.py            # 专家数据采集与特征构造
│   ├── runtime.py             # roslaunch（系统 Python 3.8）自动切换到项目 venv（Python 3.11）
│   └── nn/                    # MLP / CNN / Adam / DQN / REINFORCE
├── scripts/                   # ROS 节点（sim/slam/nav/perception/e2e/metrics/viewer/teleop）
├── launch/                    # teleop / slam_nav / perception / e2e / benchmark / bringup
├── tools/                     # 采集、训练、评测、GIF 录制
├── models/                    # 训练好的 .npz 模型
├── results/                   # 训练指标、三场景 benchmark、ROS 运行记录（JSON）
└── tests/                     # 物理 / 建图 / 规划 / 控制自测
```

## 环境要求

- Ubuntu 20.04.6 LTS + ROS Noetic（`/opt/ros/noetic`）
- Python ≥ 3.10（MuJoCo 3.1–3.2（requirements 固定 `mujoco>=3.1,<3.3`） 要求；Ubuntu 20.04 自带 3.8，因此项目使用私有 venv）
- 依赖：`mujoco>=3.1,<3.3`、`numpy==1.26.4`、`Pillow`、`PyYAML`、`rospkg`（见 `requirements.txt`）
- 无 GPU、无网络环境可完整运行（训练与评测均只用 CPU）

## 快速开始

```bash
# 0) 进入模块目录
cd src/air/uav_numpy_nn_slam

# 1) 创建项目私有 Python 环境（不修改系统 Python / ~/.bashrc）
bash main.sh setup

# 2) 编译独立 catkin 工作空间（默认 ~/uav_ws，不覆盖已有工作空间）
bash scripts/build_ros.sh
source ~/uav_ws/devel/setup.bash

# 3) 键盘遥控（物理仿真 + 禁飞区查看器）
roslaunch uav_numpy_nn_slam teleop.launch scene:=city
```

键盘（先点击 “UAV AIRSPACE” 窗口使其获得焦点）：

| 按键 | 作用 |
| --- | --- |
| `W` / `S` | 前进 / 后退 |
| `A` / `D` | 左偏航 / 右偏航 |
| `R` / `F` | 上升 / 下降 |
| `Q` / `E` | 左横移 / 右横移 |
| `Space` | 停止 |

## 运行模式

| 命令 | 说明 |
| --- | --- |
| `bash main.sh teleop city` | MuJoCo 四旋翼物理仿真 + 键盘遥控 |
| `bash main.sh slam_nav canyon` | 激光建图 + 扫描匹配 + A*（禁飞区约束）+ 纯跟踪自主导航 |
| `bash main.sh perception city` | 感知 MLP 激光去噪，发布 `/uav/scan_filtered` |
| `bash main.sh e2e airport` | 端到端 CNN：相机图像 → 速度指令 |
| `bash main.sh benchmark city` | 指标评测（运行时写 `results/ros_*.json`，不随仓库提供） |
| `bash main.sh test` | 物理 / 建图 / 规划 / 控制自测 |
| `bash main.sh train` | 重新采集数据并训练全部网络（CPU 约 15 分钟） |

## 实测结果（Ubuntu 20.04 虚拟机，4 核 CPU，无 GPU）

| 指标 | 结果 |
| --- | --- |
| 经典 SLAM + A* + 纯跟踪（三场景） | **成功率 100%**（city 35.0 s / canyon 16.3 s / airport 31.4 s，0 碰撞、0 禁飞区侵入） |
| SLAM 绝对轨迹误差 ATE RMSE | **0.298 m**（city 0.276 / canyon 0.266 / airport 0.354） |
| A* 路径长度（三场景） | 181.1 / 80.3 / 169.7 m，最小离障 1.80 / 2.20 / 2.06 m |
| 激光去噪（感知 MLP） | RMSE **7.127 → 5.635 m（−20.9%）**（source: `results/benchmark.json`） |
| 神经控制（行为克隆） | vx MAE **0.372 m/s**、yaw MAE **0.093 rad/s** |
| 神经两级控制（规划+控制） | canyon 成功率 100%，city/airport 0% |
| 端到端 CNN | vx MAE 1.29 m/s，闭环成功率 0%（见文档失败分析） |
| 强化学习对比（8 动作离散任务） | DQN 2×64 **100%** / DQN 3×64 **100%** / REINFORCE **93.3%** |
| ROS 闭环（slam_nav） | city 56.7 s / 终点误差 0.33 m / 0 碰撞；canyon 18.3 s / 0.80 m |

## 复现步骤

```bash
bash main.sh setup
bash main.sh test
bash scripts/build_ros.sh
bash main.sh slam_nav city        # 观察查看器中的地图、路径与禁飞区
bash main.sh train                # 重新训练（约 15 分钟）
.venv/bin/python tools/evaluate.py --scenes city,canyon,airport
```

## 已知限制

- 端到端 CNN 缺少全局路径记忆，闭环成功率 0%；单级神经控制在 city 场景失效（4 次碰撞），两级神经控制在 canyon 达到 100%。
- 仿真为 6-DOF 刚体 + 显式气动模型，未做真实风洞参数辨识；激光为几何射线仿真。
- 未使用 GPU，CNN 训练与推理均为 CPU 版本。

## 参考与说明

- 场景与 ROS 组织方式参考本仓库 `docs/air/` 下已有空域示例与 `docs/water/` 示例；
- 本模块所有代码、模型与数据均由本项目自行实现/生成，未使用他人代码；
- 物理引擎：MuJoCo 3.1–3.2（requirements 固定 `mujoco>=3.1,<3.3`）；ROS：Noetic；神经网络：纯 NumPy（无 PyTorch/TensorFlow）。
