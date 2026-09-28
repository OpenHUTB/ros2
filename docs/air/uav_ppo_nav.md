# 基于 PPO 的无人机自主导航与避障（uav_ppo_nav）

本模块用 **PPO（近端策略优化，Proximal Policy Optimization）** 训练一个神经网络策略，
让无人机**自主飞向目标点并避开障碍物**。训练在轻量质点环境完成（快），部署在
CarlaAir/AirSim 仿真器（真实），两侧观测/动作接口完全一致，策略可直接迁移。

## 1. 运行架构

```text
/lidar/points ─┐
               ├─► uav_ppo_nav ──/uav/cmd_vel──► carlair_ros_bridge ──► CarlaAir/AirSim
/uav/odom ─────┘   (策略推理)                     (坐标换算+执行)
```

## 2. 观测与动作

### 2.1 观测（22 维）

| 部分 | 维度 | 说明 |
|---|---|---|
| 激光雷达扇形直方图 | 16 | 360° 均分 16 个扇形（22.5°/个），每扇取最近障碍距离，归一化 `[0,1]`（1=无遮挡）。垂直方向只统计无人机高度 ±1.5 m 内的点，避免地面误报 |
| 目标相对位置 | 3 | 目标点相对无人机的机体系坐标（前/左/上），除以 6 m 归一化 |
| 机体系速度 | 3 | 前/左/上速度，分别除以 3 / 3 / 1.5 m/s 归一化 |

扇形直方图公式（机体系点 `(x, y)` 为前/左）：

```text
r   = sqrt(x² + y²)
θ   = atan2(y, x)                    # -π..π，0 = 正前方
i   = ⌊(θ + Δ/2) / Δ⌋ mod 16        # Δ = 2π/16
h[i] = min over sector i of clip(r / R_max, 0, 1)
```

### 2.2 动作（4 维，`[-1,1]`）

```text
vx = a[0] · 3.0        # 前进速度 (m/s)
vy = a[1] · 3.0        # 左移速度 (m/s)
vz = a[2] · 1.5        # 上升速度 (m/s)
ω  = a[3] · 1.0        # 偏航角速度 (rad/s)
```

直接映射到 `geometry_msgs/Twist` 的 `linear.x/y/z` 与 `angular.z`，与桥接包的
`/uav/cmd_vel` 机体系约定一致。

## 3. 训练

训练环境 `scripts/env.py` 是**质点 + 速度环**模型：`yaw += ω·dt`，
`pos += R(yaw)·[vx, -vy, vz]·dt`，随机摆放圆柱障碍物与随机目标，用射线求交生成
合成激光雷达直方图。奖励为「向目标靠近的进度 − 动作平滑 − 时间惩罚」，到达 +20、撞障碍 −20。

```shell
pip install "stable-baselines3" "gymnasium" numpy
cd src/air/uav_ppo_nav/scripts
python3 main.py --train --total 300000 --obstacles 6
# 产出 models/policy_weights.npz（部署用）与 models/best_model.zip
```

PPO 超参：`n_steps=2048, batch_size=256, lr=3e-4, gamma=0.99, gae_lambda=0.95,
ent_coef=0.005, net_arch=[64,64]`。

## 4. 部署

部署节点 `scripts/ppo_nav_node.py` 用训练导出的 MLP 权重做**纯 numpy 前向**，
因此虚拟机**不需要安装 torch / stable-baselines3**：

```shell
# 终端 1：桥接
source ~/bridge_ws/devel/setup.bash
roslaunch carlair_ros_bridge main.launch

# 终端 2：PPO 导航
source ~/bridge_ws/devel/setup.bash
roslaunch uav_ppo_nav main.launch goal_x:=30.0 goal_y:=10.0 goal_z:=-8.0
```

推理前向（MLP `[64,64]`）：

```text
h0 = tanh(W0·x + b0)
h1 = tanh(W1·h0 + b1)
a  = clip(W2·h1 + b2, -1, 1)
```

## 5. 验证

本地（无 ROS / 仿真器 / GPU）可先跑桩测试：

```shell
python3 tests/test_ppo_nav_local.py   # 53 项：直方图/坐标变换/动作映射/环境
```

训练评估（`main.py --eval --episodes 100 --obstacles 6`，30 万步训练）：

| 指标 | 数值 |
|---|---|
| 成功率 | **81/100（81.0%）** |
| 失败（碰撞/越界） | 11 |
| 超时 | 8 |
| 平均步数 | 41.1 |
| 平均奖励 | 15.6 |

真机联调在 CarlaAir 中下发目标点后，无人机自主飞行并避障（GIF 见效果图）。

## 6. 效果图

（待补：PPO 导航演示 GIF + 成功率曲线）

## 7. 参考

* [carlair_ros_bridge 桥接模块](../air/carlair_ros_bridge.md)
* [Proximal Policy Optimization (Schulman et al., 2017)](https://arxiv.org/abs/1707.06347)
* [Stable-Baselines3 文档](https://stable-baselines3.readthedocs.io/)
