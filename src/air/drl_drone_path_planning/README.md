# drl_drone_path_planning

> **开发者**：汤湘粤 (学号: 2535030067)
> 基于深度强化学习（PPO / SAC）的四旋翼无人机最优路径查找模块
> ROS 1 Noetic（catkin / roslaunch）· Python 3 · NumPy 推理 · Stable-Baselines3 训练

[中文](#1-项目简介) | [English](#english-version)

---

## 1. 项目简介

本模块把参考仓库 [`zhaoran2000/DRL-drones`](https://github.com/zhaoran2000/DRL-drones)
中的无人机强化学习环境重构为一个**可直接在 ROS 中运行**的功能包：

- 订阅 `/uav/odom` 与 `/uav/scan`，构造 28 维观测；
- 运行 PPO 或 SAC 训练得到的 Actor 策略，输出 4 维机体系速度指令；
- 发布 `/uav/cmd_vel`、飞行路径与状态文本，可直接在 RViz 中观察；
- 提供纯几何基线（航点追踪 + 激光势场）作为可复现对照；
- **部署端只依赖 NumPy**：训练完成后把 Actor MLP 导出为 `.npz`，
  ROS 机器不需要安装 PyTorch / Stable-Baselines3。

模块名没有使用宽泛的 `drl_drone_navigation`，是为了避免与仓库中已有的
`safedrl_drone_nav`、`uav_ppo_nav`、`uav_rl_benchmark` 等模块混淆。

## 2. 参考实现与本模块的边界

阅读并重构了参考仓库的两个目录：

| 参考文件 | 内容 | 本模块的对应实现 |
|---|---|---|
| `AirSimSol/dqn_drone.py` | AirSim + SB3 DQN（CNN 视觉输入） | 保留为「视觉方案」说明，不作为主路径；本模块使用轻量状态观测 |
| `Sol/Model/Environments/PBDroneEnv.py` | PyBullet 四旋翼环境，12 维运动学观测、4 维 RPM 动作、多航点奖励 | `scripts/drone_env.py`：一阶速度动力学 + 解析圆柱射线，保留多航点奖励结构 |
| `Sol/Model/PBDroneSimulator.py` | PPO / SAC / DDPG 训练编排、`SubprocVecEnv`、`EvalCallback` | `scripts/train.py`：PPO / SAC 训练与权重导出 |
| `Sol/Model/Policies/*` | 自定义 Actor-Critic 与 SB3 策略 | PPO actor 256×256 Tanh；SAC 256×256 ReLU、自动熵系数 |
| `Sol/Model/parameters.py` | 超参数默认值 | `config/drl_nav.yaml`（集中配置） |

**重构要点**：

1. 参考实现的动作是 4 个电机 RPM，需要 PyBullet 才能解释；ROS 侧更自然的是
   机体系速度，因此本模块把动作定义为 `[vx, vy, vz, yaw_rate]`，并把动力学
   抽象为一阶速度响应，训练/部署共用同一套观测与动作定义。
2. 参考实现的奖励除以 25 后量级仍偏大（进度项系数 3000/25 = 120），本模块把
   各奖励项写入 YAML 并缩放到 PPO 容易收敛的范围，逐项对应关系见 §3.3。
3. 参考实现的观测不含障碍物信息；ROS 部署必须避障，因此增加 16 扇区激光
   直方图，观测维度从 12 扩展到 28。
4. 去掉了参考实现中硬编码的 `sys.path.append` 和绝对权重路径，所有路径通过
   `$(find drl_drone_path_planning)` 与 YAML 解析。

## 3. 算法设计

### 3.1 观测（28 维）

| 下标 | 维度 | 含义 | 归一化 |
|---|---|---|---|
| 0–2 | 3 | 当前航点相对位置（机体系：前 / 左 / 上） | 除以 `goal_norm = 10 m` |
| 3–5 | 3 | roll / pitch / yaw | 除以 π |
| 6–8 | 3 | 机体线速度 | xy 除以 2 m/s，z 除以 1 m/s |
| 9–11 | 3 | 机体角速度 | 除以 π |
| 12–27 | 16 | 激光 16 扇区畅通度 | `最近距离 / 12 m`，1 = 畅通 |

所有分量裁剪到 `[-1, 1]`（激光到 `[0, 1]`），与 `observation_space` 一致。

### 3.2 动作（4 维）

```text
vx        = a[0] · max_vel_xy   = a[0] · 2.0 m/s
vy        = a[1] · max_vel_xy   = a[1] · 2.0 m/s
vz        = a[2] · max_vel_z    = a[2] · 1.0 m/s
yaw_rate  = a[3] · max_yaw_rate = a[3] · 0.8 rad/s
```

默认发布为**机体系** `geometry_msgs/Twist`（REP 103 约定）。若下游桥接要求
世界系速度，把 `config/drl_nav.yaml` 的 `command_frame` 改成 `world` 即可，
节点会自动做机体系 → 世界系旋转。

**高度保持辅助**：当目标航点与当前高度之差不超过 `altitude_hold_band = 0.3 m`
时，垂直动作衰减为 `altitude_damping = 0.1` 倍。多旋翼巡航段通常由高度环定高，
这一层既避免策略在目标高度附近学到「俯冲撞地」的局部最优，也让仿真与实机的
高度控制行为一致；训练环境和 ROS 节点调用同一个
`common.apply_altitude_hold()`，不存在两套逻辑。

### 3.3 奖励

| 奖励项 | 参考实现（除以 25 后） | 本模块默认 | 作用 |
|---|---|---|---|
| 距离塑形 | `3·exp(-2d)/25` | `0.12·exp(-2d)` | 全程提供稠密梯度 |
| 进度 | `3000·Δd/25 = 120Δd` | `5.0·Δd` | 鼓励靠近当前航点 |
| 朝向 | 机头偏差 > 10° 记 `-3/25` | `-0.05` | 保持机头指向目标 |
| 平滑 | 加速度惩罚 `/25` | `0.05` | 抑制抖动 |
| 高度 | 无（PyBullet 有重力） | `0.15·|z-z_goal|` | 抑制无意义俯冲/爬升 |
| 中间航点 | `75/25 = 3` | `5.0` | 分段引导 |
| 最终航点 | `1000/25 = 40` | `60.0` | 完成任务的强信号 |
| 碰撞 | `-10` | `-10` | 终止并重罚 |
| 时间 | 无 | `-0.01/步` | 鼓励最短路径 |

奖励结构直接对应 `PBDroneEnv._computeReward()`、`orientation_reward()` 与
`smoothness_reward()`；每个系数都在 YAML 的 `env.reward` 下可改。

### 3.4 网络与训练超参数

**PPO**（其余参数与参考 `PBDroneSimulator.setup_agent()` 对齐）

```text
policy / value 网络：256 × 256，Tanh
learning_rate 3e-4，n_steps 2048，batch 256，n_epochs 10
gamma 0.99，gae_lambda 0.95，clip_range 0.2，ent_coef 0.003
```

`ent_coef 0.003` 为实际训练使用值。

**SAC**

```text
actor / critic：256 × 256，ReLU
learning_rate 3e-4，buffer 200000，batch 256
tau 0.005，gamma 0.99，自动熵系数，train_freq / gradient_steps = 1
```

训练环境是一阶速度动力学（时间常数 0.15 s）、10 Hz 控制、500 步上限；
5 个圆柱障碍、4 个航点，起点在每回合加入小幅随机化以提升鲁棒性。

### 3.5 训练 / 部署一致性

训练环境、离线评估和 ROS 节点**共用** `scripts/common.py` 中的
`build_observation()` 与 `action_to_velocity()`，不存在两份手写实现。
SB3 的 Actor 通过 `scripts/policy.py::export_sb3_policy()` 导出为
`layer{i}_W`、`layer{i}_b`（形状与 `torch.nn.Linear.weight` 相同），
`NumpyActor` 用 `tanh(W·x+b)` 逐层复现前向计算，部署端不需要 PyTorch。

最后一层要区分算法：**PPO 是普通高斯 + `clip(mean, -1, 1)`，SAC 是 squashed
Gaussian + `tanh(mean)`**。导出文件用 `squash` 标志记录，`NumpyActor` 据此选择
映射；已用 500 组随机观测验证 NumPy 推理与 SB3 `model.predict` 的最大动作偏差
< 1e-6（不区分 squash 时偏差可达 0.24）。

## 4. 目录结构

```text
drl_drone_path_planning/
├── CMakeLists.txt
├── package.xml
├── README.md
├── requirements.txt
├── config/
│   └── drl_nav.yaml            # 强化学习超参数 + 仿真/奖励/安全配置
├── launch/
│   └── main.launch             # 一键启动
├── models/
│   ├── policy_weights.npz      # 导出的 Actor 权重（已提交，部署用）
│   ├── model.zip               # SB3 原始模型（训练产物，仓库忽略，可重新生成）
│   ├── evaluations.npz         # 训练期评估曲线数据
│   └── train_summary.json      # 训练配置与评估指标
├── rviz/
│   └── drl_nav.rviz
├── scripts/
│   ├── main.py                 # ROS 入口
│   ├── drone_env.py            # Gymnasium 训练环境
│   ├── common.py               # 配置/坐标变换/激光/观测
│   ├── policy.py               # NumPy 推理 + SB3 权重导出
│   ├── baseline.py             # 几何基线
│   ├── drl_nav_node.py         # ROS 控制节点
│   ├── train.py                # PPO/SAC 训练
│   └── evaluate.py             # 离线评估
└── tests/
    ├── test_core.py            # 13 个 NumPy/Gymnasium 单元测试
    └── test_safety.py          # 3 个激光缺失/超时安全停机测试
```

## 5. 安装

### 5.1 ROS 部署端（Ubuntu 20.04 + Noetic）

本功能包固定使用 ROS 1 Noetic 的 catkin / rospy / roslaunch 工具链。仓库中另有
ROS 2 示例，但本包不依赖 ROS 2，也不使用 colcon 或 ament。

```bash
sudo apt install python3-numpy python3-yaml python3-nose
cd ~/catkin_ws/src
ln -s /path/to/ros2/src/air/drl_drone_path_planning .
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 5.2 训练端（任意有 Python 3.9+ 的机器）

```bash
cd src/air/drl_drone_path_planning
pip install -r requirements.txt
python3 -m unittest discover -s tests -v
```

> 训练依赖 `gymnasium`、`stable-baselines3`、`torch`；部署只需要 `numpy`、`PyYAML`。
> 仓库自带的 `models/policy_weights.npz` 可以直接部署，无需重新训练。

## 6. 训练与评估

```bash
# PPO（默认）
python3 scripts/train.py --algo PPO --timesteps 1000000 --out models

# SAC
python3 scripts/train.py --algo SAC --timesteps 200000 --out models
```

训练结束会写出：

- `models/model.zip`：SB3 原始模型（可 `PPO.load()` 继续训练）；
- `models/policy_weights.npz`：纯 NumPy Actor，供 ROS 节点使用；
- `models/train_summary.json`：超参数、耗时与评估指标。

离线评估（不需要 ROS / torch）：

```bash
python3 scripts/evaluate.py --model models/policy_weights.npz --episodes 100 --baseline
```

## 7. ROS 部署

### 7.1 话题接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/uav/odom` | `nav_msgs/Odometry` | 世界系位姿 + 机体系 twist |
| 订阅 | `/uav/scan` | `sensor_msgs/LaserScan` | 2D 激光；0 rad = 机头方向 |
| 发布 | `/uav/cmd_vel` | `geometry_msgs/Twist` | 机体系速度 + `angular.z` 偏航角速度 |
| 发布 | `~path` | `nav_msgs/Path` | 已飞轨迹 |
| 发布 | `~path_waypoints` | `nav_msgs/Path` | 航点序列（latched） |
| 发布 | `~status` | `std_msgs/String` | 当前模式 / 航点 / 距离 |

话题名可在 launch 参数或 YAML 中修改。节点不依赖 TF，直接使用里程计
`header.frame_id` 作为路径坐标系（通常为 `odom`）。

### 7.2 启动

```bash
# 自动模式：有 policy_weights.npz 就用 DRL，否则退回几何基线
roslaunch drl_drone_path_planning main.launch

# 强制几何基线（用于验证接口 / 作为对照）
roslaunch drl_drone_path_planning main.launch mode:=baseline

# 单目标点覆盖 YAML 中的多航点
roslaunch drl_drone_path_planning main.launch \
  use_goal_override:=true goal_x:=4.0 goal_y:=3.0 goal_z:=2.0

# 同时打开 RViz
roslaunch drl_drone_path_planning main.launch rviz:=true
```

若使用 `carlair_ros_bridge` 之类的桥接，把 `odom_topic`、`scan_topic`、
`cmd_topic` 改成桥接实际使用的话题即可。

### 7.3 安全检查

`config/drl_nav.yaml` 的 `safety` 段提供一层与学习策略无关的保护：

- 尚未收到激光数据，或最近一帧超过 `scan_timeout = 1.0 s` 未更新时，发布零速
  `Twist` 并持续告警，不进入策略推理；收到新的实时激光数据后自动恢复；
- 正前方 ±45° 内出现 `stop_distance = 0.8 m` 以内的障碍时，平移速度清零，
  保留偏航角速度用于脱离；
- 到达最终航点后持续发布零速度并保持；
- `mode:=drl` 时如果权重缺失或观测维度不匹配会直接报错，不会静默使用随机策略。

## 8. 配置说明（`config/drl_nav.yaml`）

| 键 | 默认 | 说明 |
|---|---|---|
| `mode` | `auto` | `auto` / `drl` / `baseline` |
| `control_rate` | `10.0` | 必须与训练 `env.dt = 0.1` 对应 |
| `goal.waypoints` | 4 个点 | 世界系 ENU 航点 |
| `goal.tolerance` | `1.5 m` | 到达判定半径 |
| `limits.*` | 2.0 / 1.0 / 0.8 | 速度与偏航角速度上限 |
| `observation.sectors` | `16` | 激光扇区数（改动后需重新训练） |
| `observation.lidar_range` | `12.0 m` | 直方图量程 |
| `safety.stop_distance` | `0.8 m` | 近距刹停阈值 |
| `safety.scan_timeout` | `1.0 s` | 激光超时阈值，超时发布零速 |
| `safety.avoid_distance` | `2.0 m` | 几何基线势场作用半径 |
| `env.obstacles` | 5 个圆柱 | `[x, y, 半径, 高度]` |
| `env.altitude_hold_band` | `0.3 m` | 高度保持辅助的启用带宽 |
| `env.altitude_damping` | `0.1` | 带内的垂直动作衰减系数 |
| `env.reward.*` | 见 §3.3 | 奖励系数 |

## 9. 实验结果

在训练环境、固定随机种子 `0..99`、100 个回合下的确定性评估：

| 指标 | PPO 策略 | 几何基线 |
|---|---|---|
| 成功率 | **100%** | 100% |
| 碰撞率 | 0% | 0% |
| 超时率 | 0% | 0% |
| 平均回报 | **88.47** | 87.01 |
| 平均步数 | **70.6** | 113.6 |
| 平均路径长度 | **18.33 m** | 20.21 m |
| 平均到达航点 | 4.0 | 4.0 |

PPO 用 100 万步训练（CPU 约 678 s），部署权重为 `models/policy_weights.npz`。
与几何基线相比，策略用更少的步数和更短的路径完成同一任务；两者都 100% 成功，
说明在这个几何相对简单的场景里，学习策略主要收益是**更短的轨迹和更平滑的控制**，
而不是「基线完全做不到」。这一结论按实际数据给出，不做夸大。

> 说明：`success` 要求到达全部 4 个航点且未碰撞；`collision` 包含越界。
> 具体数字由 `scripts/evaluate.py --baseline` 实际运行得到，可用同样命令复现。

## 10. 已知限制

1. **动力学是运动学近似**：一阶速度响应不包含电机推力、姿态环和气动阻力，
   不能替代 PX4 SITL / AirSim 的物理仿真；本模块重点是策略与 ROS 接口。
2. **2D 激光 + 圆柱障碍**：只处理水平面内的圆柱，未建模楼梯、斜面和动态障碍；
   激光数据需要由仿真器或真实雷达提供。
3. **航点是预先给定的**：策略在 4 个航点之间做局部避障，不做全局 SLAM / 建图。
4. **仿真到实机存在差距**：观测做了裁剪与归一化，实机部署前应重新标定
   `lidar_range`、速度上限，并保留安全层。
5. **SAC 训练较慢**：默认给 PPO 的 100 万步预算；SAC 建议至少 20 万步，且
   在 CPU 上训练时间明显更长。

## 11. English version

**Developer:** Tang Xiangyue (student ID: 2535030067)

`drl_drone_path_planning` is a ROS 1 package for DRL-based optimal path
finding with a quadrotor. It refactors the PyBullet/SB3 environment from
[`zhaoran2000/DRL-drones`](https://github.com/zhaoran2000/DRL-drones) into a
deployable ROS module:

- **Observation (28-D):** body-frame waypoint offset (3), roll/pitch/yaw (3),
  body velocity (3), body angular velocity (3), and a 16-sector lidar
  free-space histogram.
- **Action (4-D):** body-frame `(vx, vy, vz, yaw_rate)` scaled by configured
  limits and published as `geometry_msgs/Twist`.
- **Algorithms:** PPO (256×256 Tanh actor/critic, clipped surrogate) and SAC
  (256×256 ReLU, twin critics, automatic entropy tuning), trained with
  Stable-Baselines3.
- **Deployment:** the actor MLP is exported to `models/policy_weights.npz` and
  evaluated with pure NumPy, so the ROS machine does not need PyTorch.
- **Baseline:** a parameter-free waypoint-pursuit + lidar potential-field
  controller is included for interface validation and as a reproducible
  comparison.

Quick start:

```bash
roslaunch drl_drone_path_planning main.launch            # auto: DRL or baseline
roslaunch drl_drone_path_planning main.launch mode:=baseline
python3 scripts/train.py --algo PPO --timesteps 1000000 --out models
python3 scripts/evaluate.py --model models/policy_weights.npz --episodes 100 --baseline
```

See the Chinese sections above for the full interface, reward definitions,
configuration reference and known limitations.

---

**Copyright (c) 2026 汤湘粤 (学号: 2535030067).** Licensed under the MIT License.
