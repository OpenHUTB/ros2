# rl_algorithm_compare —— MuJoCo 神经网络强化学习算法与网络结构对比

在 **MuJoCo**（OpenHUTB 课程要求的仿真平台之一）的 **CartPole（倒立摆）** 环境上，
用 **纯 NumPy 手写神经网络**（不依赖 PyTorch / stable-baselines3）实现强化学习控制，
完成两类对比：

1. **相同网络结构（MLP 16）下的不同 RL 算法**：DQN vs REINFORCE
2. **相同算法（DQN）下的不同网络结构**：MLP(16) 与 MLP(16,16)

观测为 4 维连续状态（小车位置/速度、摆杆角度/角速度），输出 2 个离散控制力
（左推/右推），构成"状态 → 控制指令"的端到端神经网络策略。

## 依赖

```bash
sudo apt install python3-pip
python3 -m pip install --user mujoco==2.3.7 numpy
```

> mujoco 需 2.3.x（兼容 Python3.8）。本模块不依赖 torch/gym/sb3，纯 NumPy 实现。

## 运行

### 方式一：launch 启动（推荐）

```bash
colcon build --packages-select rl_algorithm_compare
source install/setup.bash
ros2 launch rl_algorithm_compare main.launch.py
```

### 方式二：直接运行脚本

```bash
# 训练 3 个对比项
python3 train.py --algo dqn       --layers 16    --tag dqn_mlp1      --episodes 600
python3 train.py --algo dqn       --layers 16,16 --tag dqn_mlp2      --episodes 600
python3 train.py --algo reinforce --layers 16    --tag reinforce_mlp1 --episodes 600

# 评估
python3 evaluate.py --model results/dqn_mlp1.pkl --algo dqn --episodes 20

# 汇总对比（学习曲线图 + 对比表）
python3 compare.py --out results

# 渲染推理 gif
python3 render_gif.py --model results/dqn_mlp1.pkl --algo dqn --episodes 1
```

## 结果

- `results/*.pkl`：训练好的神经网络策略
- `results/*.csv`：训练过程的平均回报曲线
- `results/*.gif`：策略推理动图
- `results/compare_curve.png`：学习曲线对比图
- `results/compare_summary.csv`：性能对比汇总表
