# reacher_track —— MuJoCo 机械臂神经网络目标跟踪

在 **MuJoCo**（OpenHUTB 课程要求仿真平台之一）的 **Reacher**（2 关节平面机械臂）环境中，
用**纯 NumPy 手写神经网络**（不依赖 PyTorch / stable-baselines3）实现
**"感知 → 神经网络控制 → 闭环目标跟踪"**：

- **感知**：关节编码器（2 关节角 + 三角函数编码）+ 目标检测（目标相对末端的坐标差），构成 6 维感知状态；
- **控制**：一个 MLP 神经网络（6→128→128→2）直接由感知状态映射到两个关节的扭矩；
- **训练**：先用「逆运动学 + PD」专家控制器生成演示数据，**监督学习**训练 MLP 逼近专家控制律，
  再用 **DAgger（数据集聚合）迭代**收集神经网络自身轨迹并用专家标注，缓解监督学习的"分布偏移"，
  使部署后的闭环稳定跟踪目标。

与课程中 CartPole 的 RL 算法对比作业（DQN vs REINFORCE、Double/Dueling DQN）**任务不同**：
本模块是机械臂目标跟踪的**感知-控制闭环**，而非强化学习算法对比。

## 环境

- 模型：2 关节平面机械臂（上臂 0.15m、前臂 0.12m），MuJoCo（RK4，步长 0.02s），见 `models/reacher.xml`
- 感知状态（6 维）：`[sin(q1), cos(q1), sin(q2), cos(q2), dx, dy]`
  - `q1,q2`：肩、肘关节角（编码器）
  - `dx,dy`：目标点相对末端点的全局坐标差（目标检测）
- 动作（2 维）：肩、肘关节扭矩，范围 `[-2, 2]`
- 控制目标：末端（fingertip）到达目标点（target），末端误差最小

## 依赖

```bash
python3 -m pip install --user mujoco==2.3.7 numpy matplotlib pillow
```

## 数据采集 / 训练 / DAgger / 评估

```bash
cd ~/ros2_ws/src/reacher_track

# 1. IK+PD 专家生成演示数据 -> demo.npz
python3 reacher_track/collect_demo.py

# 2. 监督训练 MLP -> mlp.npz + 训练损失
python3 reacher_track/train_mlp.py

# 3. DAgger 迭代缓解分布偏移 -> 更新 mlp.npz
python3 reacher_track/dagger_iter.py

# 4. 闭环部署评估 -> 误差曲线 + 性能表
python3 reacher_track/deploy_eval.py

# 5. 渲染目标跟踪动图 -> reacher_ep0.gif / reacher_ep1.gif
python3 reacher_track/render_gif.py
```

## launch 启动（完整流水线）

```bash
cd ~/ros2_ws/src/reacher_track
ros2 launch reacher_track main.launch.py
```

依次执行 collect_demo → train_mlp → dagger_iter → deploy_eval。

## 性能评价（30 回合闭环，误差 <0.05m 记为成功）

| 方法 | 平均末端误差(m) | 中位误差(m) | 成功率 | 平均步数 |
|---|---|---|---|---|
| **MLP 神经网络(DAgger)** | **0.0331** | **0.0179** | **83.3%** | 77.4 |
| IK+PD 专家(基线) | 0.0194 | 0.0195 | 100.0% | 39.2 |

动图 `docs/img/reacher_track/reacher_ep0.gif` 展示了机械臂末端误差从约 0.20m 收敛到约 0.03m 的目标跟踪过程。

## 目录结构

```
src/reacher_track/
├── package.xml / setup.py / setup.cfg
├── models/reacher.xml            # MuJoCo 机械臂物理模型
├── reacher_track/
│   ├── reacher_env.py            # MuJoCo 环境封装 + 正/逆运动学 + PD 控制器
│   ├── nn.py                     # 纯 NumPy MLP + Adam
│   ├── collect_demo.py           # IK+PD 专家生成演示数据
│   ├── train_mlp.py              # 监督训练 MLP
│   ├── dagger_iter.py            # DAgger 迭代(分布偏移缓解)
│   ├── deploy_eval.py            # 闭环部署评估 + 误差曲线
│   └── render_gif.py             # matplotlib 渲染跟踪动图(无显示器兜底)
├── launch/main.launch.py
├── config/algo_config.yaml
├── results/                      # 评估结果数据（eval_summary.csv）
└── README.md
```

## 说明

- 本模块为**监督学习**的感知-控制闭环（非 RL），与课程 CartPole RL 算法对比作业任务不同、互不冲突。
- 依赖仅 MuJoCo + NumPy + matplotlib，可在 ROS2 Foxy/Humble + Python3.8/3.10 运行。
- gif 渲染用 matplotlib（Agg 后端），无需显示器。
