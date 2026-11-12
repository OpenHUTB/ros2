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

### 方式二：一键运行（推荐）

```bash
cd ~/ros2_ws/src/rl_algorithm_compare
python3 -m rl_algorithm_compare.main
# 安装后也可：ros2 run rl_algorithm_compare main
# 默认回合数：DQN 300，REINFORCE 600（收敛较慢）
```

### 方式三：直接运行脚本

脚本已做包路径引导（`sys.path`），可直接运行：

```bash
cd ~/ros2_ws/src/rl_algorithm_compare/rl_algorithm_compare

# 训练 3 个对比项
python3 train.py --algo dqn       --layers 16    --tag dqn_mlp1      --episodes 300
python3 train.py --algo dqn       --layers 16,16 --tag dqn_mlp2      --episodes 300
python3 train.py --algo reinforce --layers 16    --tag reinforce_mlp1 --episodes 600

# 评估
python3 evaluate.py --model results/dqn_mlp1.pkl --algo dqn --episodes 20

# 汇总对比（学习曲线图 + 对比表）
python3 compare.py --out results

# 渲染推理 gif（无 OpenGL 环境自动降级到 matplotlib 绘制）
python3 render_gif.py --model results/dqn_mlp1.pkl --algo dqn --episodes 1

# 多随机种子基准评测（均值 ± 标准差，结果写 results/benchmark/）
python3 benchmark.py --seeds 0,1,2 --episodes 200
```

`train.py` 也支持 `--seed` 复现单次训练；训练循环已抽成 `train_agent()`，供 `benchmark.py` 复用。

## 基准评测

单次训练曲线抖动很大，`benchmark.py` 对每组配置跑多个随机种子：

```bash
cd src/rl_algorithm_compare
python3 -m rl_algorithm_compare.benchmark --seeds 0,1 --episodes 120
python3 -m unittest discover -s test      # 19 个用例，不需要 MuJoCo
```

输出 `results/benchmark/summary.csv`（各配置均值/标准差/最值）与 `curves.csv`（跨种子平均曲线），
该目录已在 `.gitignore` 中忽略。

> 种子同时控制**环境初值、ε-greedy 探索、回放采样和网络初始权重**（`MLP(seed=…)`）。
> 建议至少跑 5 个种子：初始化方差很可观，2 个种子容易得出"稳定收敛"的假象
> （实测一层 DQN：固定初始化 std≈0.8，纳入初始化后 std≈39.9）。

耗时参考：2 个种子 × 120 回合（3 组配置）在本机约 3 分钟；只跑 DQN 两组配置约 2 分钟。
回合数越大越慢，CI 里建议 `--episodes 20` 起步。

## 结果

- `results/*.pkl`：训练好的神经网络策略
- `results/*.csv`：训练过程的平均回报曲线
- `results/*.gif`：策略推理动图
- `results/compare_curve.png`：学习曲线对比图
- `results/compare_summary.csv`：性能对比汇总表
