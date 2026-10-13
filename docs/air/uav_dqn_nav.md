# 基于 DQN 的无人机自主导航与避障（uav_dqn_nav）

本模块用 **DQN（Deep Q-Network，深度 Q 网络）** 训练一个 Q 网络，让无人机
**自主飞向目标点并避开障碍物**。它与 [uav_ppo_nav（PPO 避障）](./uav_ppo_nav.md)
共用**同一训练环境、同一观测、同一动作接口**，仅把强化学习算法从 PPO 换成 DQN，
用于课程"相同模型不同 RL 算法对比"。

## 1. 运行架构

```text
/lidar/points ─┐
               ├─► uav_dqn_nav ──/uav/cmd_vel──► carlair_ros_bridge ──► CarlaAir/AirSim
/uav/odom ─────┘   (Q 网络推理)                    (坐标换算+执行)
```

## 2. 观测与动作（与 PPO 完全一致，全部世界系 ENU）

> 坐标系约定继承自 uav_ppo_nav：CarlaAir 的 SimpleFlight 偏航保持不生效，
> 因此观测与动作全部使用**世界系（ENU）**，机头漂移不影响轨迹。

### 2.1 观测（22 维）

| 部分 | 维度 | 说明 |
|---|---|---|
| 激光雷达扇形直方图 | 16 | 360° 均分 16 个扇形（22.5°/个，**扇形 0 = 正东**，逆时针），每扇取最近障碍距离，归一化 `[0,1]`（1=无遮挡） |
| 目标相对位置 | 3 | 目标点相对无人机的**世界系**偏移（东/北/天），除以 8 m 归一化 |
| 世界系速度 | 3 | 东/北/天速度，分别除以 3 / 3 / 1.5 m/s 归一化 |

扇形直方图公式（以无人机为原点，世界系点 `(x, y)` 为东/北）：

```text
r   = sqrt(x² + y²)
θ   = atan2(y, x)                    # -π..π，0 = 正东
i   = ⌊(θ + Δ/2) / Δ⌋ mod 16        # Δ = 2π/16
h[i] = min over sector i of clip(r / R_max, 0, 1)      # R_max = 12 m
```

### 2.2 动作（DQN 离散化：11 个）

DQN 是离散动作算法，把 PPO 的连续 4 维速度空间离散成 11 个动作：

| 索引 | 动作 | 世界系速度 (vx, vy, vz) |
|:---:|---|---|
| 0-7 | 水平 8 方向（东/东北/北/西北/西/西南/南/东南） | 2.1 m/s 巡航，保持高度 |
| 8 | 上升 | vz = +1.5 m/s |
| 9 | 下降 | vz = -1.5 m/s |
| 10 | 悬停 | (0, 0, 0) |

动作经 `act_index_to_velocity(idx)` 映射为与 PPO 相同的 `[vx, vy, vz, yaw_rate]`
世界系速度，再发布为 `geometry_msgs/Twist` 的 `linear.x/y/z` 与 `angular.z`。
**因为是世界系速度，桥接必须以 `body_frame:=false` 启动**：

```shell
roslaunch carlair_ros_bridge main.launch publish_image:=false body_frame:=false
```

> 节点启动时自动校验桥接的 `control/body_frame` 参数：读到 `true` 打印 `logerr`，
> 读不到 `logwarn` 提示，避免"无人机会飞但方向不对"的隐蔽故障。

## 3. 算法原理（DQN）

DQN 用神经网络近似最优动作价值函数 Q\*(s, a)，通过最小化**时序差分（TD）误差**
学习：

```text
L(θ) = E[( r + γ · max_a' Q_target(s', a'; θ⁻) − Q(s, a; θ) )²]
```

其中 θ⁻ 是**目标网络**参数（软更新 θ⁻ ← τθ + (1−τ)θ⁻，τ = 0.005），γ = 0.99 为折扣因子。

### 3.1 网络结构

```text
obs(22) → Linear(64) → ReLU → Linear(64) → ReLU → Linear(11)
                                                    └─ 每个动作的 Q 值
```

总参数量约 5.9K，CPU 上单次前向 < 1 ms，10 Hz 实时推理远富余。

### 3.2 强化学习技巧

| 技巧 | 实现 |
|---|---|
| 经验回放（Replay Buffer） | 环形队列容量 50000，随机采样打破样本相关性 |
| 目标网络（Target Network） | 软更新 τ = 0.005，缓解自举发散 |
| ε-greedy 探索 | ε 从 1.0 线性衰减到 0.05（10 万步） |
| 梯度裁剪 | max_grad_norm = 10，稳定训练 |
| 每 N 步训练 | update_freq = 4，batch = 128 |

### 3.3 训练流程

```text
1. 观测构造：点云 → 16 扇区直方图 + 目标 + 速度 = 22 维
2. ε-greedy 选离散动作 a（概率 ε 随机，否则 argmax Q(s,a)）
3. 动作映射为世界系速度，环境 step 一步，得 (s, a, r, s', done)
4. 存入经验回放池；每 4 步采样一个 batch 更新 Q 网络
5. 目标网络软更新；每 10000 步用贪心策略评估成功率，保存最优模型
```

### 3.4 训练环境

训练环境 `scripts/env.py` 与 PPO **逐行一致**（质点 + 速度环一阶滞后）：

```text
vel += (cmd_vel − vel) · dt / τ          # τ = 0.45 s
pos += vel · dt
```

随机摆放圆柱障碍物与随机目标，用射线-圆求交生成合成激光雷达直方图。
奖励 = 「向目标靠近的进度 − 动作平滑 − 时间惩罚」，到达 +20、撞障碍/越界 −20。

```shell
pip install numpy gymnasium torch --index-url https://download.pytorch.org/whl/cpu
cd src/air/uav_dqn_nav/scripts
python3 main.py --train --total 200000 --obstacles 6 --out ../models
# 产出 models/policy_weights.npz（部署用）与 models/best_model.pt
```

## 4. 部署

部署节点 `scripts/dqn_nav_node.py` 用训练导出的**纯 numpy 权重**做前向推理
（3 层 MLP），虚拟机**不需要安装 torch**：

```shell
# 终端 1：桥接（必须 body_frame:=false，因为策略输出世界系速度）
source ~/bridge_ws/devel/setup.bash
roslaunch carlair_ros_bridge main.launch publish_image:=false body_frame:=false

# 终端 2：DQN 导航
source ~/bridge_ws/devel/setup.bash
roslaunch uav_dqn_nav main.launch goal_x:=16.6 goal_y:=5.4 goal_z:=-10.0
```

推理前向（Q 网络，ReLU 激活）：

```text
h0 = relu(W0·x + b0)
h1 = relu(W1·h0 + b1)
q  = W2·h1 + b2
a  = argmax(q)
```

## 5. 验证

### 5.1 训练环境评估

`main.py --eval --weights ../models/policy_weights.npz --episodes 100 --obstacles 6`：

| 指标 | 数值 |
|---|---|
| 成功率 | 待填写（虚拟机训练后填入） |
| 失败（碰撞/越界） | 待填写 |
| 超时 | 待填写 |
| 平均步数 | 待填写 |
| 平均奖励 | 待填写 |

> 评估口径与 PPO 完全一致，可直接对比两种算法在同一环境上的表现。

### 5.2 与 PPO 的对比（同一环境，同一观测/动作接口）

| 指标 | DQN (uav_dqn_nav) | PPO (uav_ppo_nav) |
|---|---|---|
| 成功率 | 待填写 | 78/100（78.0%） |
| 平均步数 | 待填写 | 38.4 |
| 平均奖励 | 待填写 | 16.5 |

> 训练步数不同（DQN 默认 20 万步，PPO 默认 30-40 万步），对比时建议取相同预算。

## 6. 效果图

![DQN 自主导航演示](img/air/uav_dqn_nav/dqn_nav_demo.gif)

上图：DQN 策略驱动 CarlaAir 无人机自主飞向目标点（第一视角，前视相机）。
（gif 用 ScreenToGif 录制，<10 MB）

## 7. 参考

* [carlair_ros_bridge 桥接模块](./carlair_ros_bridge.md)
* [基于 PPO 的无人机导航避障（对比模块）](./uav_ppo_nav.md)
* Mnih et al., "Human-level control through deep reinforcement learning",
  *Nature* 518:529-533, 2015
* [DQN 原理解读（Nature 论文）](https://www.nature.com/articles/nature14236)
