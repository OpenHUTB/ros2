# rl_dqn_improvement —— DQN 改进算法对比（MuJoCo CartPole）

在 **MuJoCo**（OpenHUTB 课程要求的仿真平台之一）的 CartPole（倒立摆）环境中，
用**纯 NumPy 手写神经网络**（不依赖 PyTorch / stable-baselines3）实现并对比三种
DQN 系算法：

- **DQN**：标准深度 Q 网络（基线），用目标网络取 max 计算 TD 目标；
- **Double DQN**：在线网络选择最优动作、目标网络评估，缓解 Q 值过估计；
- **Dueling DQN**：网络输出分裂为状态价值 V 与优势 A 两个流，更稳定地学习价值。

三者共享经验回放、目标网络与 epsilon-greedy 探索，差异仅在 TD 目标构造或网络
输出结构上，从而在同一环境上公平对比算法改进。

## 环境

- 观测：4 维（小车位置/速度、摆角/角速度）
- 动作：2 个离散（左推/右推，力 ±20N）
- 奖励：每存活一步 +1；目标撑满 200 步
- 物理：MuJoCo（RK4，步长 0.01s），模型见 `models/cartpole.xml`

## 依赖

```bash
python3 -m pip install --user mujoco==2.3.7 numpy matplotlib pillow
```

## 一键运行

```bash
python3 -m rl_dqn_improvement.main
# 安装后也可直接：ros2 run rl_dqn_improvement main
```

## 训练 / 评估 / 对比

```bash
cd ~/ros2_ws/src/rl_dqn_improvement

# 训练三个对比项
python3 -m rl_dqn_improvement.train --variant dqn     --tag dqn_mlp1
python3 -m rl_dqn_improvement.train --variant double  --tag double_mlp1
python3 -m rl_dqn_improvement.train --variant dueling --tag dueling_mlp1

# 评估
python3 -m rl_dqn_improvement.evaluate --model results/dqn_mlp1.pkl

# 汇总对比曲线与表格
python3 -m rl_dqn_improvement.compare --out results --tags dqn_mlp1 double_mlp1 dueling_mlp1

# 渲染推理 gif
python3 -m rl_dqn_improvement.render_gif --model results/dqn_mlp1.pkl
```

## launch 启动

```bash
cd ~/ros2_ws
colcon build --packages-select rl_dqn_improvement
source install/setup.bash
ros2 launch rl_dqn_improvement main.launch.py variant:=double tag:=double_mlp1 episodes:=300
```

## 目录结构

```
src/rl_dqn_improvement/
├── package.xml / setup.py / setup.cfg
├── models/cartpole.xml        # MuJoCo 环境模型
├── rl_dqn_improvement/
│   ├── cartpole_env.py        # MuJoCo 环境封装
│   ├── nn.py                  # NumPy MLP + Adam
│   ├── dqn.py                 # DQN / Double / Dueling 三变体
│   ├── train.py               # 训练 + 记录曲线
│   ├── evaluate.py            # 评估模型
│   ├── compare.py             # 汇总对比曲线与表格
│   └── render_gif.py          # 渲染推理 gif（matplotlib 兜底）
├── launch/main.launch.py
├── config/algo_config.yaml
└── README.md
```

## 说明

- gif 渲染优先 MuJoCo 离屏渲染（需 OpenGL）；无显示器环境自动回退 matplotlib 状态动画。
- 依赖仅 MuJoCo + NumPy + matplotlib，可在 ROS2 Foxy/Humble + Python3.8/3.10 运行。
