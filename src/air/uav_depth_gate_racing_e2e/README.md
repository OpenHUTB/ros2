# uav_depth_gate_racing_e2e — 深度图端到端穿越门竞速

对应作业任务（4）端到端决策与（2）传感器感知。对比实验②：
**同一 RL 算法、不同网络结构**。

输入是**原始深度图**，不加任何人工特征：网络自己决定"下一个门在哪、该往哪偏"。
主路径是**专家示范 + 水平镜像增广的行为克隆**；DAgger 在线聚合与 PPO 微调都已
实现并有测试覆盖，但在本任务的数据量下**没有跑赢行为克隆**——两条负结果连同
诊断记在文档 §7.2 / §7.3。网络、反向传播、PPO、DAgger 全部纯 numpy 手写，
不依赖 torch / stable-baselines3。

依赖 [uav_mujoco_sim](../uav_mujoco_sim) 提供 `gate_track` 场景与
`/camera/depth`、`/uav/odom`、`/uav/cmd_vel`。

---

## 1. 与课程已有模块的区别

| 已有模块 | 区别 |
|---|---|
| `uav_ppo_nav` | 观测是 22 维**人工特征**（16 线雷达直方图 + 目标相对位置 + 世界系速度），训练环境是**质点 + 一阶滞后**模型，训练用 torch/stable-baselines3 |
| `uav_keyboard_control` | 人在回路，没有学习 |
| `octree_uav_3d_pathfinding` | 几何规划，不是学习式决策 |

|  | `uav_ppo_nav` | 本模块 |
|---|---|---|
| 观测 | 22 维手工特征 | **48×64 原始深度图 + 机体系速度** |
| 训练环境 | 质点模型（纯 numpy） | **MuJoCo 刚体动力学**（转子饱和、混控、姿态环） |
| 算法 | PPO | **DAgger + PPO** |
| 框架 | torch + sb3 | **纯 numpy**（含手写卷积反向） |
| 任务 | 点到点导航避障 | 穿越门竞速（5 m 间距、±1.1 m 开口） |

## 2. 架构

```
深度图 48×64 ─┐
              ├─→ 主干(mlp/cnn/gru/attn) ─→ 128 维 ─→ 策略头 μ ─→ tanh ─→ 速度指令
机体系速度 3  ─┘                                  └─→ 价值头 V
                                                            │
                                       航向系速度 ─→ 参考轨迹 ─→ 几何控制器 ─→ 四桨推力
```

四种主干共用同一套输入输出契约，只换中间结构（对比实验②）：

| arch | 主干 | 说明 |
|---|---|---|
| `mlp` | 深度图拉平 → 全连接 | 无空间先验的基线 |
| `cnn` | 3 层 stride-2 卷积 → 展平 | 有平移等变性 |
| `gru` | CNN → GRU | 带记忆，可抑制单帧噪声 |
| `attn` | CNN → 单头自注意力 | 全局感受野 |

## 3. 安装

运行环境：ROS2 Humble（Ubuntu 20.04，Python 3.8）+ MuJoCo 3.12.0（仿真侧 Python ≥3.10）。

```bash
cd ~/uav_ws/ros2_ws/src
cp -r <本目录> .

# 仿真侧（需要 Python ≥3.10）
uv venv ~/uav_ws/.venv --python 3.12
uv pip install --python ~/uav_ws/.venv/bin/python 'mujoco==3.12.0' numpy

# ROS 侧（Python 3.8）
cd ~/uav_ws/ros2_ws
colcon build --packages-select uav_mujoco_sim uav_depth_gate_racing_e2e
source install/setup.bash
```

## 4. 运行步骤

```bash
# 自检（不需要 ROS）
python main.py --selftest

# 主路径：专家示范（含镜像增广）→ 行为克隆。
# --rounds 0 = 只做第 0 轮；实测 DAgger 的后续轮次没有正收益（见文档 §7.2）
python main.py --dagger --arch cnn --rounds 0 --episodes 8 --jitter 1.0 \
               --bc-epochs-first 100 --out data/policy_bc.npz

# PPO 微调（可选；实测这点预算下没有正收益，见文档 §7.3）
python main.py --ppo --init data/policy_bc.npz --iterations 6 \
               --log-std-init -2.0 --out data/policy_ppo.npz --hist data/ppo_hist.json

# DAgger 实验：跑 3 轮纠错，每轮打印闭环通过门数
python main.py --dagger --arch cnn --rounds 3 --episodes 8 --bc-epochs 25 \
               --bc-epochs-first 40 --out /tmp/dagger.npz

# 评估与出图
python main.py --evaluate --model data/policy_bc.npz --episodes 10 --jitter 0.6 \
               --out results.json
python main.py --figures  --model data/policy_bc.npz --hist data/hist.json \
               --compare compare_arch.json \
               --outdir ../docs/img/air/uav_depth_gate_racing_e2e

# 对比实验②：同一 PPO、四种网络结构（示范数据只采一次）
python main.py --compare-structures --episodes 8 --bc-epochs 40 \
               --iterations 6 --out compare_arch.json --model-out data/policy_cnn.npz
```

ROS2 在线运行：

```bash
# 仿真 + 桥接 + 竞速节点
ros2 launch uav_depth_gate_racing_e2e racing.launch.py with_sim:=true

# 只起竞速节点（假定 /camera/depth 与 /uav/odom 已在运行）
ros2 launch uav_depth_gate_racing_e2e racing.launch.py
```

## 5. 话题接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/camera/depth` | sensor_msgs/Image（32FC1，米） | 策略唯一的外部感知输入 |
| 订阅 | `/uav/odom` | nav_msgs/Odometry | 机体系速度观测 |
| 发布 | `/uav/cmd_vel` | geometry_msgs/Twist | 机体系速度指令 |
| 发布 | `/racing/status` | std_msgs/String | 推理次数、频率、延迟、当前指令 |
| 发布 | `/racing/depth_view` | sensor_msgs/Image（mono8） | 网络实际看到的深度图 |

**坐标系**：训练时动作是**航向系**速度（与训练环 `GateRacingEnv` 一致），
底座的 `VelocityCommander` 按**实时全姿态**旋到世界系。节点发布前先做

```
v_body = Rᵀ · R_z(yaw) · a
```

底座再左乘 R，复原出的世界速度正好是 `R_z(yaw)·a`。不做这一步，机身为了
前进俯仰 20° 时约三分之一的前向指令会被旋到竖直轴上。

## 6. 文件结构

```
uav_depth_gate_racing_e2e/
├── main.py                       入口（作业要求以 main 开头）
├── uav_depth_gate_racing_e2e/
│   ├── cli.py                    专家 / DAgger / PPO / 评估 / 对比 / 出图 / 自检
│   ├── selftest.py               29 项自检，无 ROS
│   ├── plotting.py               配图
│   ├── core/
│   │   ├── nn.py                 手写 Conv2d / GRU / 自注意力 / LayerNorm / Adam
│   │   ├── gate_env.py           竞速环境（深度图观测 + 门通过判定）
│   │   ├── expert.py             比例控制专家（DAgger 的标签来源）
│   │   ├── policy.py             四种主干 + 高斯策略头 + 价值头
│   │   ├── bc.py                 行为克隆（MSE / Huber）
│   │   ├── dagger.py             DAgger：策略跑、专家纠错、轮次加权聚合
│   │   └── ppo.py                PPO：GAE + 裁剪代理目标 + tanh 雅可比
│   └── ros/racing_node.py        ROS2 在线节点
├── config/racing.yaml            参数
├── launch/racing.launch.py       启动入口
├── data/                         预训练策略权重
└── test/                         pytest
```

## 7. 实测结果

见 `docs/air/uav_depth_gate_racing_e2e.md`。

## 8. 已知限制

- 仿真频率 20 Hz（策略）× 100 Hz（控制器）。深度图 48×64 是训练速度与
  空间分辨率的折中；再大一圈单轮训练时间线性上升。
- 专家是比例控制器，只吃真值位姿，上界为"干净通过全部门"。
- **DAgger 在本任务上没跑赢"只做一轮行为克隆"**（数据见文档 §7.2）。原因是
  第 0 轮的行为克隆本身还没到能提供有效纠错样本的程度：策略一出门就出专家
  包络，采到的新数据极少。DAgger 已实现并有测试覆盖，详见文档 §7.2。
- 环境的动作语义是**航向系**速度而不是机体系：映射用实时姿态会形成正反馈
  （机身一倾斜映射跟着转，姿态环为了追上又倾得更多），竞速里几个门下来
  直接翻转。ROS 节点里的坐标变换就是为对齐这一点。
- 混控的力矩预算是 `min(4T_max−T, T)`，期望推力顶到上限时预算归零、姿态失控。
  控制器里限了总推力（`thrust_alloc`）。
- 奖励权重：前进项按帧累加、过门项只发一次，配错时"直着飞不穿门"与"全穿门"
  得分相当（文档 §3.3 有量级表）。
