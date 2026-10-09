# uav_ppo_nav

基于 **PPO（近端策略优化）** 的无人机**自主导航与避障**模块。

- **观测（22 维）**：激光雷达 16 方向扇形直方图（16）+ 目标相对位置（3）+ 世界系速度（3）
- **动作（4 维）**：世界系（ENU）速度 `[vx, vy, vz, yaw_rate]`，归一化到 `[-1,1]`
- **训练**：轻量质点环境（纯 numpy，`scripts/env.py`），几分钟训完
- **部署**：CarlaAir/AirSim 仿真器，经 `carlair_ros_bridge` 走 ROS 话题，纯 numpy 推理（无需 torch/sb3）

> **为什么用世界系而不是机体系？** 实测 CarlaAir 里 SimpleFlight 的偏航保持不生效
> （即使命令 `yaw_rate=0`，机头仍以约 9°/s 漂移并 ±25° 摆动）。机体系速度控制会让
> 前进方向随漂移的机头一起转，轨迹变成**绕圈**。改用世界系后机头怎么转都不影响轨迹。
> **因此桥接必须用 `body_frame:=false` 启动。**
>
> 启动时节点会**自动校验**：若读到桥接参数 `control/body_frame=true`（或读不到），
> 会打印明确的 `logerr` / `logwarn` 提示，而不是让你对着"乱飞"的无人机排查。
>
> 另外桥接的 `control/cmd_duration`（单条速度指令时长）默认已改为 **0.1 s**，
> 与本模块训练环境的 `dt` 一致；设大了会造成过冲震荡。

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
python3 main.py --train --total 400000 --obstacles 16 --out ../models
# 产出 models/policy_weights.npz（部署用）+ models/best_model.zip
```

> `--out ../models` 不能省：训练脚本默认写当前目录下的 `models`，而部署读的是
> 包内的 `models/policy_weights.npz`。

### 2. 评估（可选）

```bash
python3 main.py --eval --weights ../models/policy_weights.npz --episodes 100 --obstacles 16
```

### 3. 部署（Ubuntu 20.04 + ROS Noetic + 虚拟机，无需 torch/sb3）

```bash
# 终端 1：桥接（发布 /uav/odom、/lidar/points，执行 /uav/cmd_vel）
#         必须 body_frame:=false：策略输出的是世界系速度
source ~/bridge_ws/devel/setup.bash
roslaunch carlair_ros_bridge main.launch publish_image:=false body_frame:=false

# 终端 2：PPO 导航（自动飞向 nav/goal 指定的目标点并避障）
source ~/bridge_ws/devel/setup.bash
roslaunch uav_ppo_nav main.launch goal_x:=16.6 goal_y:=5.4 goal_z:=-10.0
```

## 观测 / 动作接口（训练与部署完全一致，全部世界系 ENU）

| 部分 | 维度 | 含义 |
|---|---|---|
| 激光雷达直方图 | 16 | 每个 22.5° 扇形内最近障碍距离，归一化 `[0,1]`（1=无遮挡）；扇形 0 = 正东，逆时针 |
| 目标相对位置 | 3 | 目标点相对无人机的世界系偏移（东/北/天），除以 8 m 归一化 |
| 世界系速度 | 3 | 东/北/天速度，除以各自上限归一化 |
| **动作** | 4 | `[vx, vy, vz, yaw_rate]`，映射到 `[±3, ±3, ±1.5] m/s` 与 `±1 rad/s` |

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `nav/goal` | `[4, 3, -27]` | 目标点（ENU 世界系） |
| `rate/publish_hz` | 10.0 | 策略推理/发布频率（须与训练 `dt=0.1 s` 一致） |
| `topic/cmd_vel` | `/uav/cmd_vel` | 速度指令话题（世界系速度） |
| `nav/weights` | `models/policy_weights.npz` | 策略权重路径 |

## 本地测试（无需 ROS / 仿真器 / GPU）

```bash
python3 tests/test_ppo_nav_local.py        # 19 项：直方图/世界系变换/动作映射/环境/策略
python3 tests/test_ppo_nav_node_local.py   # 20 项：部署节点（mock 掉 rospy）
```

## 实测结果

| 场景 | 成功率 / 误差 |
|---|---|
| 训练环境评估（100 回合，16 障碍） | **78%** 成功（碰撞/越界 17、超时 5），平均 38.4 步 |
| CarlaAir 真机（6.71 m 目标） | 3.4 s 到达，误差 0.59 m |
| CarlaAir 真机（8.94 m 目标） | 4.2 s 到达，误差 0.38 m |
| CarlaAir 真机（16.12 m 目标） | 6.4 s 到达，**误差 0.22 m** |

> 评估须带 `--obstacles 16`：`--eval` 的默认值是 6，不传会得到不同的成功率。

## 说明

- 训练环境是**质点 + 速度环一阶滞后**模型（`vel += (cmd−vel)·dt/τ`，τ = 0.45 s），
  接口（观测/动作）与真实部署完全一致；建模滞后是为了消除真机上的过冲震荡。
- 部署侧只依赖 numpy（策略推理是纯 numpy 前向），避免在虚拟机上安装 torch。
- **只提交部署必需的 `models/policy_weights.npz`（25 KB）**；`best_model.zip` /
  `final_model.zip` 是 SB3 训练产物（各约 165 KB，pickle 压缩包），不入库，训练脚本可重新生成。
- 训练总步数 40 万，但导出的是**验证集最优检查点**（`best_model.zip`，约 30 万步处）——
  这是 `train.py` 的设计（评估集更优者胜出），`policy_weights.npz` 与 `best` 权重逐位一致。
- 对比实验（同模型不同 RL 算法 / 同算法不同结构）见后续 PR。
