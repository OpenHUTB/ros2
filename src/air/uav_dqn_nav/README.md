# uav_dqn_nav —— 基于 DQN 的无人机自主导航与避障

[![ROS](https://img.shields.io/badge/ROS-Noetic-green)](https://wiki.ros.org/noetic)

本模块用 **DQN（Deep Q-Network）** 训练无人机在三维空间中从起点自主飞向目标点，
途中避开随机圆柱障碍物。它是 OpenHUTB 空域载具板块中 **uav_ppo_nav（PPO）的算法对比模块**：
两者共用**同一训练环境、同一 22 维观测、同一世界系（ENU）动作接口**，仅把策略学习算法
从 PPO 替换为 DQN，用于课程要求的"相同模型不同 RL 算法对比"。

## 1. 模块特性

| 项目 | 说明 |
| --- | --- |
| 算法 | DQN（深度 Q 网络，Mnih et al., Nature 2015） |
| 网络 | 双隐藏层 MLP（22 → 64 → 64 → 35），ReLU 激活 |
| 观测 | 22 维：16 扇区激光雷达直方图 + 目标相对位置(3) + 速度(3)（全部世界系 ENU） |
| 动作 | 35 个离散动作：16 方向巡航 + 16 方向慢速逼近 + 上升/下降 + 悬停 |
| 强化学习技巧 | 经验回放、目标网络（软更新 τ=0.005）、ε-greedy 线性衰减、梯度裁剪 |
| 训练环境 | 纯 numpy 质点 + 速度环一阶滞后（与 PPO 完全相同） |
| 部署 | ROS Noetic 节点，纯 numpy 推理（无需 torch） |

## 2. 目录结构

```
uav_dqn_nav/
├── CMakeLists.txt          # catkin 构建配置
├── package.xml             # ROS 包描述
├── README.md
├── config/
│   └── dqn_nav.yaml        # 部署参数（话题、频率）
├── launch/
│   └── main.launch         # 统一 launch 启动入口
├── models/                 # 训练输出：best_model.pt / policy_weights.npz
└── scripts/
    ├── env.py              # 训练环境（与 uav_ppo_nav/env.py 逐行一致）
    ├── policy.py           # 观测/动作共享逻辑 + DQN 离散动作表 + Q 网络 + numpy 推理
    ├── dqn_agent.py        # DQN 算法：经验回放、目标网络、ε-greedy、更新
    ├── train.py            # 训练入口（main.py --train）
    ├── eval.py             # 评估入口（main.py --eval）
    ├── dqn_nav_node.py     # ROS 部署节点
    └── main.py             # 统一入口
```

## 3. 运行环境

- Ubuntu 20.04（`lsb_release -a` 确认）
- ROS Noetic（ROS 1），catkin 工作空间
- Python 3.8+，依赖：`numpy`、`gymnasium`、`torch`（仅训练需要）、`rospy` 等 ROS 标准包
- CarlaAir / AirSim 仿真器（部署时），桥接包 `carlair_ros_bridge`

安装训练依赖（在虚拟机内，中国大陆网络建议加清华源加速）：

```bash
sudo apt update && sudo apt install -y python3-pip
pip install numpy gymnasium -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install torch --index-url https://download.pytorch.org/whl/cpu
```

验证环境（Ubuntu 20.04 / ROS Noetic）：

```bash
lsb_release -a                       # 确认 Ubuntu 20.04
source /opt/ros/noetic/setup.bash && echo $ROS_DISTRO   # 确认 noetic
python3 -c "import numpy, gymnasium, torch; print('deps ok')"
```

> 说明：训练侧依赖 torch（CPU 版即可，网络很小）；部署节点只用 numpy，无 torch 依赖。

## 4. 快速开始

### 4.1 训练 DQN 策略（在虚拟机内，40 万步与 PPO 对齐）

```bash
cd ~/catkin_ws/src/uav_dqn_nav
python3 main.py --train --total 400000 --epsilon-decay 250000 --obstacles 16 --out models35
```

训练过程每 15000 步评估一次成功率（贪心策略），最优模型与纯 numpy 权重自动保存到
`--out` 指定目录（`best_model.pt`、`policy_weights.npz`）。训练完成后屏幕会打印
类似输出（训练时 stdout 即完整日志）：

```
step=  105000 | eps=0.601 | eval成功=1/20 (5.0%) | 失败=13 超时=6 | 平均奖励=-21.53 | 平均步数=183.9
step=  390000 | eps=0.050 | eval成功=0/20 (0.0%) | 失败=2 超时=18 | 平均奖励=-22.79 | 平均步数=366.6
训练完成：best 成功率 = 1/20，权重已导出 models35/policy_weights.npz
```

### 4.2 评估

```bash
python3 main.py --eval --weights models35/policy_weights.npz --episodes 100 --obstacles 16
```

输出口径与 uav_ppo_nav 一致（成功率 / 失败 / 超时 / 平均步数 / 平均奖励），
可直接与 PPO 结果对比（见第 7 节）。

### 4.3 部署到 CarlaAir/AirSim（ROS Noetic）

1. 启动仿真器（Windows 上运行 `AbandonedPark.exe` / AirSim）；
2. 启动桥接（**必须 body_frame:=false**，本节点发布的是世界系速度）：

```bash
roslaunch carlair_ros_bridge main.launch publish_image:=false body_frame:=false
```

3. 启动 DQN 导航节点：

```bash
roslaunch uav_dqn_nav main.launch goal_x:=16.6 goal_y:=5.4 goal_z:=-10.0
```

> 目标点是**绝对 ENU 坐标**，请写无人机当前位置附近可达的点；
> 不指定时用 launch 内默认值（4.0, 3.0, -27.0）。

## 5. 训练参数（`main.py --train`）

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--total` | 200000 | 总训练步数 |
| `--batch` | 128 | 回放采样批量 |
| `--buffer` | 50000 | 经验回放容量 |
| `--gamma` | 0.99 | 折扣因子 |
| `--lr` | 1e-3 | 学习率 |
| `--tau` | 0.005 | 目标网络软更新系数 |
| `--update-freq` | 4 | 每 N 步训练一次 |
| `--epsilon-start` | 1.0 | 初始探索率 |
| `--epsilon-end` | 0.05 | 最终探索率 |
| `--epsilon-decay` | 100000 | ε 线性衰减步数 |
| `--obstacles` | 6 | 障碍物数量 |
| `--eval-freq` | 10000 | 评估间隔（步） |
| `--eval-episodes` | 10 | 每次评估回合数 |
| `--out` | models | 输出目录 |
| `--seed` | 42 | 随机种子 |

## 6. 算法原理（简述）

DQN 用神经网络近似最优动作价值函数 Q\*(s, a)，目标是最小化时序差分误差：

```
L(θ) = E[( r + γ·max_a' Q_target(s', a'; θ⁻) - Q(s, a; θ) )²]
```

训练流程：

1. 观测构造：激光雷达点云 -> 16 扇区直方图，拼接目标相对位置与速度得 22 维观测；
2. ε-greedy 选动作：概率 ε 随机探索，否则取 argmax_a Q(s, a)；
3. 与环境交互一步，把 (s, a, r, s', done) 存入经验回放池；
4. 每 4 步从回放池采样一个 batch，用目标网络计算 TD 目标并更新 Q 网络；
5. 目标网络每步软更新：θ⁻ ← τ·θ + (1-τ)·θ⁻；
6. 每 10000 步用贪心策略评估成功率，保存最优模型。

## 7. 与 PPO（uav_ppo_nav）的对比结果

> 两模块在同一虚拟机（Ubuntu 20.04 / ROS Noetic，6 核 8.6 GB）上分别完成训练，
> 评估口径完全一致（100 回合 / 贪心策略 / 目标半径 0.4 m / 速度环一阶滞后），可直接对比。

### 7.1 训练环境评估（16 障碍，DQN 与 PPO 默认设置）

| 指标 | DQN (uav_dqn_nav) | PPO (uav_ppo_nav) |
| --- | --- | --- |
| 成功率（100 回合） | **4%**（4/100） | **78%**（78/100） |
| 失败（碰撞/越界） | 8 | 17 |
| 超时 | 88 | 5 |
| 平均步数 | 363.2 | 38.4 |
| 平均奖励 | -19.2 | — |
| 训练步数 | 400000（ε 衰减 250000） | 400000（PPO 默认） |

### 7.2 训练环境评估（6 障碍，压力更小的场景）

| 指标 | DQN (uav_dqn_nav) | PPO (uav_ppo_nav) |
| --- | --- | --- |
| 成功率（100 回合） | 0%（0/100） | 待补跑 |
| 失败（碰撞/越界） | 64 | — |
| 超时 | 36 | — |
| 平均步数 | 200.8 | — |
| 平均奖励 | -27.0 | — |

### 7.3 结果分析与结论

DQN 能学会**安全避障**（16 障碍下 100 回合仅 8 次碰撞），但**无法精确停靠
目标点**（超时占 88%）：离散动作（22.5° 方向网格 + 巡航/慢速两档）在接近目标时
的横向误差（0.19-0.4 m）与到达半径（0.4 m）同量级，无人机常在目标附近绕圈。
PPO 的连续动作空间可以输出任意方向速度，平滑逼近并停靠目标，因此成功率显著更高。
这组对比直接展示了**动作空间离散化对精细控制任务（点目标到达）的性能影响**——
这也是本作业"端到端模型/算法对比"的核心结论。

### 7.4 运行演示（16 障碍，DQN 策略实时飞行）

![DQN 运行演示 gif](../../../docs/img/air/uav_dqn_nav/dqn_demo.gif)

> 红色点为无人机（按 DQN 策略实时选动作），灰色圆为圆柱障碍，绿色圈为目标。
> 本回合 219 步内成功避开全部障碍但停在目标附近（超时），直观复现
> "能避障但无法精确停靠 0.4 m 点目标"的对比结论。

![DQN 训练曲线](../../../docs/img/air/uav_dqn_nav/training_curve.png)

![DQN 飞行轨迹（6 障碍）](../../../docs/img/air/uav_dqn_nav/trajectory.png)

> 权重：`models/models35/policy_weights.npz`、`models/models6/policy_weights.npz`（训练日志由 train.py 运行时 stdout 输出，不入库）。

## 8. 注意事项

- 本模块与 uav_ppo_nav 的坐标系约定一致：**全部使用世界系（ENU）**，
  桥接必须 `body_frame:=false`，否则无人机会往错误方向飞；
- 训练环境刻意保持与 PPO 完全相同（同一 NavEnv），对比结果才有意义；
- 部署侧推理为纯 numpy（前向 3 层 MLP），10 Hz 实时性远富余。
