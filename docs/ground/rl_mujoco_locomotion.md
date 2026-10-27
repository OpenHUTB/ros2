# MuJoCo Locomotion 强化学习对比实验

## 项目简介

本项目在 **MuJoCo 3.1.6** 物理引擎上，针对三个经典连续控制 locomotion 环境——**HalfCheetah（猎豹奔跑）**、**Ant（蚂蚁四足爬行）**、**Walker2d（二足行走）**——实现了 **PPO** 与 **SAC** 两种强化学习算法的纯 NumPy 手写训练与对比实验。

与仓库已有项目相比：
- **环境不同**：仓库已有 RL 项目使用 CartPole（倒立摆）、Unitree G1 人形机器人、无人机等；本项目首次引入 MuJoCo 原生 HalfCheetah / Ant / Walker2d 三个地面 locomotion 环境。
- **算法不同**：仓库已有 PPO 用于无人机巡航与 G1 平衡；本项目首次引入 **SAC** 算法，并与 PPO 形成 3 环境 × 2 算法 = 6 组对比矩阵。
- **实现方式**：全部使用 **NumPy 手写神经网络与反向传播**，不依赖 PyTorch / TensorFlow / Stable-Baselines3。

## 算法原理

### PPO（Proximal Policy Optimization）

PPO 是一种 on-policy 策略梯度算法，核心目标是限制每次策略更新的幅度，防止训练崩溃。其 clipped surrogate objective 为：

![PPO clipped surrogate objective](../img/ground/rl_mujoco_locomotion/eq_ppo_clip.png)

其中策略概率比 r_t(θ) = π_θ(a_t | s_t) / π_θold(a_t | s_t)，Â_t 为广义优势估计（GAE）：

![GAE 优势估计](../img/ground/rl_mujoco_locomotion/eq_ppo_gae.png)

![TD 误差](../img/ground/rl_mujoco_locomotion/eq_ppo_td.png)

本项目使用 Actor-Critic 架构，Actor 输出动作均值（tanh 压缩到 [-1, 1]），Critic 输出状态价值。为保证训练稳定，对 `log_std` 做了 [-4, 1] 夹紧，对策略/价值梯度做了范数裁剪，并在 KL 超过阈值 `target_kl` 时提前结束本轮更新。

### SAC（Soft Actor-Critic）

SAC 是一种 off-policy 最大熵强化学习算法，在最大化累计奖励的同时最大化策略熵，以鼓励探索：

![SAC 最大熵目标](../img/ground/rl_mujoco_locomotion/eq_sac_objective.png)

其中 α 为温度系数，按目标熵 -|A| 自动调节（log α 夹紧在 [ln 0.01, ln 10]）。

SAC 使用双 Q 网络（Double Q-learning）缓解过估计，Actor 通过重参数化技巧采样动作：

![SAC 重参数化采样](../img/ground/rl_mujoco_locomotion/eq_sac_reparam.png)

> **实现说明**：本项目 SAC 的 actor 梯度采用了**简化推导**（未严格还原 tanh 重参数化的完整 Jacobian 项），因此推理效果与 Stable-Baselines3 等成熟实现存在差距，详见"结果分析"一节。

## 环境说明

| 环境 | 状态维度 | 动作维度 | 任务目标 |
|------|---------|---------|---------|
| HalfCheetah | 17 | 6 | 单足猎豹沿 x 轴快速奔跑 |
| Ant | 27 | 8 | 四足蚂蚁保持平衡并向前移动 |
| Walker2d | 17 | 6 | 二足机器人稳定行走 |

三个环境的 MJCF 模型文件均来自 gymnasium 的 MuJoCo 环境，已复制到 `assets/` 目录，运行时不依赖 gymnasium。

奖励函数统一为：

```
reward = forward_reward(前进速度) - ctrl_cost(控制代价) - contact_cost(接触冲击代价) + healthy(存活奖励)
```

其中 HalfCheetah 不做提前终止；Ant 与 Walker2d 在机体高度/姿态超出健康范围时提前终止，单集上限 1000 步（与 Gym 默认一致）。

## 快速开始

### 环境要求

- Ubuntu 20.04 / 22.04（实测 20.04.6 LTS）
- Python 3.8+（实测 3.8.10）
- MuJoCo 3.1.6（无头渲染使用 `MUJOCO_GL=egl`，已内置在 `env.py` 中）
- ROS 2（可选，仅用于 launch 入口）

### 安装依赖

```bash
pip install -r src/rl_mujoco_locomotion/requirements.txt
```

### 一键检查环境

```bash
python3 src/rl_mujoco_locomotion/main.py doctor
```

### 快速验证（50 回合，约数分钟）

```bash
python3 src/rl_mujoco_locomotion/main.py train --env halfcheetah --alg ppo --episodes 50 --max-steps 20000
python3 src/rl_mujoco_locomotion/main.py render --env halfcheetah --alg ppo
```

### 完整训练（6 组实验）

`--max-steps` 是**总环境步数预算**，到达后自动停止，避免不同环境单集长度差异导致的训练时间失控（Walker2d/Ant 会提前终止，若只按回合数计数会跑出极多回合）。

```bash
# 6 组实验，每组 30 万环境步
for env in halfcheetah ant walker2d; do
  for alg in ppo sac; do
    python3 src/rl_mujoco_locomotion/main.py train --env $env --alg $alg \
      --episodes 1000000 --max-steps 300000 --seed 0
  done
done

# 生成 6 组学习曲线对比图
python3 src/rl_mujoco_locomotion/main.py compare
```

### 渲染推理 GIF

```bash
# --stride 表示每隔 N 个环境步保留一帧；单集很短的环境（如 Walker2d）用 1，长集环境用 5~20
python3 src/rl_mujoco_locomotion/main.py render --env halfcheetah --alg ppo --episodes 2 --stride 5
python3 src/rl_mujoco_locomotion/main.py render --env ant --alg sac --episodes 2 --stride 5
python3 src/rl_mujoco_locomotion/main.py render --env walker2d --alg ppo --episodes 20 --stride 1
```

### ROS 2 启动

```bash
cd <ros2_workspace>
colcon build --packages-select rl_mujoco_locomotion
source install/setup.bash
ros2 launch rl_mujoco_locomotion main.launch.py env:=halfcheetah alg:=ppo episodes:=50
```

## 实验结果

> 以下全部为**实测数据**（Ubuntu 20.04.6 / Python 3.8.10 / MuJoCo 3.1.6，seed=0，每组 30 万环境步），不是估计值。
> 表中"训练末段回报"取训练 CSV 最后 10 集的平均回报；"确定性评估"为加载最终模型后按确定性策略（不采样噪声）跑 2~20 集的平均回报，即 GIF 中展示的策略表现。

### 学习曲线

![Learning Curves](../img/ground/rl_mujoco_locomotion/comparison.png)

### 推理 GIF

| HalfCheetah-PPO | HalfCheetah-SAC |
|:---:|:---:|
| ![halfcheetah-ppo](../img/ground/rl_mujoco_locomotion/halfcheetah_ppo.gif) | ![halfcheetah-sac](../img/ground/rl_mujoco_locomotion/halfcheetah_sac.gif) |

| Ant-PPO | Ant-SAC |
|:---:|:---:|
| ![ant-ppo](../img/ground/rl_mujoco_locomotion/ant_ppo.gif) | ![ant-sac](../img/ground/rl_mujoco_locomotion/ant_sac.gif) |

| Walker2d-PPO | Walker2d-SAC |
|:---:|:---:|
| ![walker2d-ppo](../img/ground/rl_mujoco_locomotion/walker2d_ppo.gif) | ![walker2d-sac](../img/ground/rl_mujoco_locomotion/walker2d_sac.gif) |

### 性能对比

| 环境 | 算法 | 训练预算 | 训练末段回报 | 确定性评估回报 | 评估平均集长 |
|------|------|---------|------------|--------------|------------|
| HalfCheetah | PPO | 30 万步 | 399.7 | 132.0 ± 35.9 | 1000 |
| HalfCheetah | SAC | 30 万步 | **645.8** | **661.7 ± 1.1** | 1000 |
| Ant | PPO | 30 万步 | -443.4 | -356.5 ± 170.7 | 58 |
| Ant | SAC | 30 万步 | **149.4** | **158.2 ± 19.3** | 1000 |
| Walker2d | PPO | 30 万步 | -4.1 | -5.0 ± 0.3 | 8 |
| Walker2d | SAC | 约 8 万步（中止） | -10.4 | -9.0 ± 0.2 | 8 |

### 结果分析

1. **HalfCheetah：SAC 明显优于 PPO。** SAC 的训练回报和确定性评估回报都更高（661.7 vs 132.0），且评估方差极小（±1.1），说明策略已较稳定；PPO 训练回报不低（末段 399.7），但确定性评估只有 132.0，说明策略对探索噪声依赖较大、确定性动作并不稳定。
2. **Ant：SAC 能维持站立并缓慢前进，PPO 未能学会。** SAC 单集能跑满 1000 步并取得正的评估回报（158.2），但前向速度很低，GIF 中位移很小；PPO 的回报长期为负且单集只有约 58 步，策略频繁倒地。
3. **Walker2d：两种算法在本实现与预算下均未收敛。** 两者单集都只坚持 8 步左右就倒地（GIF 中可以看到反复跌倒），回报接近甚至低于"原地站立"的水平。原因分析：
   - Walker2d 对姿态要求最严格（健康判定为 z ∈ [0.8, 2.0]、躯干倾角 ±1.0 rad），早期随机策略几乎必然出界终止，正样本极少；
   - 30 万步预算远小于该任务通常需要的百万级步数；
   - 本项目的 SAC actor 梯度为简化推导，探索不足，进一步限制了收敛。
4. **整体结论：** 在"纯 NumPy + 轻量 MLP + 30 万步预算"的约束下，SAC 在 HalfCheetah 与 Ant 上均优于 PPO；Walker2d 需要更长训练或更完善的 SAC 梯度实现，本仓库暂未达到可用水平（如实记录，未做美化）。

## 目录结构

```
src/rl_mujoco_locomotion/
├── main.py                 # 统一入口：doctor/test/train/evaluate/compare/render
├── requirements.txt        # mujoco / numpy / matplotlib / Pillow / imageio
├── package.xml             # ROS 2 包描述
├── CMakeLists.txt          # ament_cmake + ament_python 构建规则
├── launch/
│   └── main.launch.py      # ROS 2 launch 入口（参数：env/alg/episodes）
├── assets/                 # HalfCheetah / Ant / Walker2d 的 MJCF 模型
├── rl_mujoco_locomotion/
│   ├── __init__.py
│   ├── nn.py               # NumPy MLP + Adam 优化器
│   ├── env.py              # HalfCheetah / Ant / Walker2d 环境封装与 EGL 渲染
│   ├── ppo.py              # PPO 算法实现
│   ├── sac.py              # SAC 算法实现
│   ├── train.py            # 训练主循环（支持 --max-steps 步数预算）
│   ├── evaluate.py         # 确定性评估
│   ├── render_gif.py       # 推理 GIF 渲染
│   └── compare.py          # 6 组对比图汇总
└── results/                # 训练产出（CSV、权重、GIF、对比图），已加入 .gitignore
```

## 源码解析

### 神经网络（`nn.py`）

- `MLP`：多层感知机，隐藏层 ReLU 激活，输出层线性（PPO Actor 用 tanh 压缩）
- `Adam`：标准 Adam 优化器，一阶矩 + 二阶矩估计
- `soft_update` / `hard_update`：目标网络软更新与硬更新
- `clip_grad_norm`：梯度范数裁剪，用于稳定 PPO 训练

### 环境封装（`env.py`）

- `BaseLocomotionEnv`：统一封装 MuJoCo 的 `MjModel` / `MjData`，提供 `reset()` / `step()` / `render()`
- 每个环境独立的奖励函数（前进奖励 - 控制代价 - 接触代价 + 存活奖励）
- 健康状态检测（z 轴高度范围、躯干角度范围），不健康即提前终止
- `render()` 使用 `MUJOCO_GL=egl` 无头渲染，缓存 `Renderer` 实例，并用跟随躯干（torso）的自由相机，保证录屏画面中机器人始终居中可见

### PPO（`ppo.py`）

- `RolloutBuffer`：收集 on-policy 轨迹，计算 GAE 与 returns
- `PPO.act()`：tanh 高斯策略采样（`log_std` 夹紧）
- `PPO.update()`：clipped surrogate objective + 价值函数回归 + target-KL 早停 + 梯度裁剪

### SAC（`sac.py`）

- `ReplayBuffer`：off-policy 经验回放池
- `SAC.act()`：tanh 重参数化采样
- `SAC.update()`：双 Q 网络 + 软目标更新 + 自动温度调节

## 测试验证

### 测试环境

| 项目 | 版本 |
|------|------|
| OS | Ubuntu 20.04.6 LTS |
| Python | 3.8.10 |
| MuJoCo | 3.1.6 |
| NumPy | 1.24.4 |
| imageio | 2.35.1 |
| ROS 2 | Humble（用于 colcon 构建验证） |

### 验证步骤

```bash
# 1. 环境检查
python3 src/rl_mujoco_locomotion/main.py doctor
# 2. 单元测试（环境 reset/step 冒烟）
python3 src/rl_mujoco_locomotion/main.py test
# 3. 快速训练验证
python3 src/rl_mujoco_locomotion/main.py train --env halfcheetah --alg ppo --episodes 50 --max-steps 20000
# 4. 评估与渲染
python3 src/rl_mujoco_locomotion/main.py evaluate --env halfcheetah --alg ppo
python3 src/rl_mujoco_locomotion/main.py render --env halfcheetah --alg ppo --episodes 2 --stride 5
# 5. 6 组 30 万步完整训练 + 对比图
for env in halfcheetah ant walker2d; do for alg in ppo sac; do
  python3 src/rl_mujoco_locomotion/main.py train --env $env --alg $alg --episodes 1000000 --max-steps 300000
done; done
python3 src/rl_mujoco_locomotion/main.py compare
# 6. ROS 2 构建验证
cd <ros2_workspace>
source /opt/ros/humble/setup.bash
colcon build --packages-select rl_mujoco_locomotion
```
