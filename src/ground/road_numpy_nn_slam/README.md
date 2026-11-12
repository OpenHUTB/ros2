# RoadNN：城市道路地面载具仿真、激光 SLAM 与纯 NumPy 神经网络导航

本模块是 `OpenHUTB/ros2` 仓库的**地面载具（城市道路自动驾驶车）**示例，覆盖课程作业「地面载具」选题从物理仿真到神经网络闭环导航的完整实现：

1. **物理仿真 + 键盘运动控制**：2-D 自行车（单轨）动力学模型——加速/制动/滚阻/风阻/不足转向，`W/A/S/D/Space` 键盘遥控；
2. **传感器感知与运动控制**：72 束 360° 激光雷达（含噪声/野值/丢点）、GNSS、IMU、前视相机；感知 MLP 对激光去噪后再送入控制器；
3. **建图与导航**：对数几率占据栅格 + 相关性扫描匹配 SLAM；A* 全局规划（道路 + 障碍约束）、Stanley 横向控制、纵向速度 PID、红绿灯停车与行人礼让规则；
4. **端到端模型**：前视相机（48×64×3）→ 2 层 CNN → `(throttle, steer)`，行为克隆训练；
5. **神经网络训练 / 测试 / ROS 封装**：纯 NumPy 实现 MLP/CNN/Adam/DQN/REINFORCE，7 个 `.npz` 模型、8 个 ROS 节点、6 个 launch；
6. **多场景性能评价**：`downtown`（城市路网 + 红绿灯 + 行人）、`campus`（园区环路）、`construction`（施工封路）三场景闭环 benchmark，含闯红灯、撞行人、越界时间指标与 RL 算法/网络结构对比。

完整原理、算法与实验数据见文档页：[`docs/ground/road_numpy_nn_slam.md`](../../../docs/ground/road_numpy_nn_slam.md)。

## 目录结构

```
road_numpy_nn_slam/
├── main.sh                       # 统一入口（teleop / slam_nav / perception / e2e / benchmark / test / train / setup）
├── road_numpy_nn_slam/           # 核心 Python 包（纯 Python + NumPy）
│   ├── config.py                 # 车辆参数、传感器、场景、红绿灯周期
│   ├── scenes.py                 # downtown / campus / construction 路网、信号灯、行人、停放车
│   ├── sim.py                    # 自行车动力学 + 激光/GNSS/IMU/相机 + 碰撞/闯红/行人计数
│   ├── render.py                 # 俯视图渲染（道路/车道线/信号灯/行人/雷达/HUD）
│   ├── slam.py                   # 占据栅格 + 扫描匹配 SLAM
│   ├── planning.py               # A*（道路 + 障碍） + DWA 局部规划
│   ├── control.py                # Stanley 横向、速度 PID、红绿灯停车、行人礼让
│   ├── datasets.py               # 专家数据采集与特征构造
│   ├── runtime.py                # roslaunch 系统 Python -> 项目 venv 自动切换
│   └── nn/                       # MLP / CNN / Adam / DQN / REINFORCE
├── scripts/                      # ROS 节点（sim/slam/nav/perception/e2e/metrics/viewer/teleop）
├── launch/                       # teleop / slam_nav / perception / e2e / benchmark / bringup
├── tools/                        # 采集、训练、评测、GIF 录制
├── models/                       # 训练好的 .npz 模型
├── results/                      # 训练指标、三场景 benchmark、ROS 运行记录（JSON）
└── tests/                        # 物理 / 建图 / 规划 / 控制自测
```

## 环境要求

- Ubuntu 20.04.6 LTS + ROS Noetic（`/opt/ros/noetic`）
- Python ≥ 3.8（项目私有 venv；仅需 NumPy，无 GPU、无 MuJoCo/CARLA 依赖）
- 依赖：`numpy`、`Pillow`、`PyYAML`、`rospkg`（见 `requirements.txt`）

## 快速开始

```bash
cd src/ground/road_numpy_nn_slam

# 1) 创建项目私有 Python 环境（不修改系统 Python / ~/.bashrc）
bash main.sh setup

# 2) 编译独立 catkin 工作空间（默认 ~/road_ws，不覆盖已有工作空间）
bash scripts/build_ros.sh
source ~/road_ws/devel/setup.bash

# 3) 键盘遥控（物理仿真 + 红绿灯/行人查看器）
roslaunch road_numpy_nn_slam teleop.launch scene:=downtown
```

键盘（先点击 “ROAD CAR” 窗口使其获得焦点）：

| 按键 | 作用 |
| --- | --- |
| `W` | 油门（加速） |
| `S` | 刹车 / 停车后倒车 |
| `A` / `D` | 左转 / 右转 |
| `Space` | 刹车 |

## 运行模式

| 命令 | 说明 |
| --- | --- |
| `bash main.sh teleop downtown` | 自行车模型车辆物理仿真 + 键盘遥控（红绿灯/行人查看器） |
| `bash main.sh slam_nav construction` | 激光建图 + 扫描匹配 + A* + Stanley 自主导航 |
| `bash main.sh perception downtown` | 感知 MLP 激光去噪，发布 `/car/scan_filtered` |
| `bash main.sh e2e campus` | 端到端 CNN：相机图像 → 油门/转向 |
| `bash main.sh benchmark downtown` | 指标评测（写 `results/ros_*.json`） |
| `bash main.sh test` | 物理 / 建图 / 规划 / 控制自测 |
| `bash main.sh train` | 重新采集数据并训练全部网络 |

## 实测结果（Ubuntu 20.04 虚拟机，4 核 CPU，无 GPU）

| 指标 | 结果 |
| --- | --- |
| 经典 SLAM + A* + Stanley（三场景） | **成功率 100%**（downtown 34.8 s / campus 43.5 s / construction 34.4 s，0 碰撞、0 闯红灯、0 撞行人、0 越界） |
| SLAM 绝对轨迹误差 ATE RMSE | **0.255 m**（地图 1400×1400 @0.10 m，可行驶区覆盖率 59.4%） |
| A* 路径长度（三场景） | 120.0 / 133.0 / 140.9 m，最小离障 4.23 / 18.20 / 1.71 m，规划 24 / 1317 / 282 ms |
| 激光去噪（感知 MLP） | RMSE **6.542 → 2.070 m（+68.4%）**，有效波束 +50.9% |
| 神经控制（行为克隆） | steer MAE **0.064 rad**（目标 ≤0.10 ✓），throttle MAE 0.237（目标 ≤0.20） |
| 神经两级控制（规划+控制） | downtown 成功率 50%，campus/construction 0% |
| 端到端 CNN | throttle MAE 0.136（✓）、steer MAE 0.211（≈均值基线 0.209）；闭环 0/3 |
| 强化学习对比（离散道路任务） | DQN 2×64 **66.7%** / DQN 3×64 **50.0%** / REINFORCE **43.3%** |
| ROS 闭环（slam_nav） | downtown 52.9 s / 终点误差 1.95 m；construction 37.8 s / 1.79 m（0 碰撞、0 闯红、0 行人） |

## 复现步骤

```bash
bash main.sh setup
bash main.sh test
bash scripts/build_ros.sh
bash main.sh slam_nav downtown     # 观察查看器中的建图、路径、红绿灯与行人
bash main.sh train                 # 重新采集并训练（CPU 约 15 分钟）
.venv/bin/python tools/evaluate.py --scenes downtown,campus,construction
```

## 已知限制

- control 的 throttle MAE（0.237）略高于 0.20 的设计目标，主要来自验证集中停车/起步瞬态样本；
- 端到端 CNN 的 steer MAE 与“预测均值”基线相当：单帧前视相机无法推断 A* 路线的转向意图，闭环 0/3；
- campus 场景对学习型控制器较难（起步处有行人横穿），经典 MissionController 可稳定通过；
- 激光去噪与规划都是几何/统计模型，未做真实车辆参数辨识。

## 参考与说明

- 本仓库已有地面载具示例（CARLA、差速底盘、纯跟踪等）作为文档与组织方式参考；
- 本模块所有代码、模型与数据均由本项目自行实现/生成，未使用他人代码；
- 神经网络为纯 NumPy（无 PyTorch/TensorFlow），可在无 GPU 的普通虚拟机中完整训练与评测。
