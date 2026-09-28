# uav_ppo_nav

基于 **PPO（近端策略优化）** 的无人机**自主导航与避障**模块。

- **观测（22 维）**：激光雷达 16 方向扇形直方图（16）+ 目标相对位置（3）+ 机体系速度（3）
- **动作（4 维）**：机体系速度 `[vx, vy, vz, yaw_rate]`，归一化到 `[-1,1]`
- **训练**：轻量质点环境（纯 numpy，`scripts/env.py`），几分钟训完
- **部署**：CarlaAir/AirSim 仿真器，经 `carlair_ros_bridge` 走 ROS 话题，纯 numpy 推理（无需 torch/sb3）

## 运行架构

```text
/lidar/points ─┐
               ├─► uav_ppo_nav ──/uav/cmd_vel──► carlair_ros_bridge ──► CarlaAir/AirSim
/uav/odom ─────┘   (策略推理)                      (坐标换算+执行)
```

## 快速开始

### 1. 训练（Windows 或任意有 torch/sb3 的机器）

```bash
pip install "stable-baselines3" "gymnasium" numpy
cd scripts
python3 main.py --train --total 300000 --obstacles 6
# 产出 models/policy_weights.npz（部署用）+ models/best_model.zip
```

### 2. 评估（可选）

```bash
python3 main.py --eval --weights ../models/policy_weights.npz --episodes 100
```

### 3. 部署（Ubuntu 20.04 + ROS Noetic + 虚拟机，无需 torch/sb3）

```bash
# 终端 1：桥接（发布 /uav/odom、/lidar/points，执行 /uav/cmd_vel）
source ~/bridge_ws/devel/setup.bash
roslaunch carlair_ros_bridge main.launch

# 终端 2：PPO 导航（自动飞向 nav/goal 指定的目标点并避障）
source ~/bridge_ws/devel/setup.bash
roslaunch uav_ppo_nav main.launch goal_x:=30.0 goal_y:=10.0 goal_z:=-8.0
```

## 观测 / 动作接口（训练与部署完全一致）

| 部分 | 维度 | 含义 |
|---|---|---|
| 激光雷达直方图 | 16 | 每个 22.5° 扇形内最近障碍距离，归一化 `[0,1]`（1=无遮挡） |
| 目标相对位置 | 3 | 目标点相对无人机的位置（机体系，除以 6 m 归一化） |
| 机体系速度 | 3 | 前/左/上速度，除以各自上限归一化 |
| **动作** | 4 | `[vx, vy, vz, yaw_rate]`，映射到 `[±3, ±3, ±1.5] m/s` 与 `±1 rad/s` |

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `nav/goal` | `[30, 10, -8]` | 目标点（ENU） |
| `rate/publish_hz` | 10.0 | 策略推理/发布频率 |
| `topic/cmd_vel` | `/uav/cmd_vel` | 速度指令话题 |

## 本地测试（无需 ROS / 仿真器 / GPU）

```bash
python3 tests/test_ppo_nav_local.py   # 53 项：直方图/坐标变换/动作映射/环境
```

## 说明

- 训练环境是**质点 + 速度环**模型，接口（观测/动作）与真实部署完全一致，训练出的
  策略可直接迁移到 CarlaAir；这是"快速训练 + 真实部署"的常见做法。
- 部署侧只依赖 numpy（策略推理是纯 numpy 前向），避免在虚拟机上安装 torch。
- 对比实验（同模型不同 RL 算法 / 同算法不同结构）见后续 PR。
