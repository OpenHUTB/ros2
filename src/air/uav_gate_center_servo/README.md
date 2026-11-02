# uav_gate_center_servo — 单门深度视觉伺服

对应作业任务（2）传感器感知与（4）端到端决策。

输入 48×64 深度图，块平均到 12×16；一个 192→32→3 的小 MLP 回归门框中心
`[forward, left, up]`，比例控制器输出航向系速度穿过单个门。推理只用深度图；
标签只在训练时由仿真真值生成。

## 与已有模块的区别

| 模块 | 做法 | 本模块不同 |
|---|---|---|
| `uav_depth_gate_racing_e2e` | 深度图直接映射到速度；BC/DAgger/PPO；CNN/GRU/注意力 | 显式回归门中心，再用 P 控制；单门；无策略网络/RL |
| `openhutb_yolo_mlp_control` | RGB + YOLO 检测框 + 视觉居中 | 深度图、门框几何中心、无 YOLO |
| 本模块 | 感知与控制分开，可解释 | #410 的可解释基线/消融 |

## 架构

```text
48×64 深度 ──▶ 12×16 块平均 ──▶ 192→32→3 MLP ──▶ [forward,left,up]
                                                      │
                                          比例伺服 ──▶ 航向系速度 (m/s)
                                                      │
                                          几何控制器 ──▶ 四旋翼
```

- 标签：`gate_center - (p + R·c_b)`，`c_b=(0.075, 0, 0.012)` 是相机安装偏移。
- 图像右 = 机体 -y，图像上 = 机体 +z；投影 `u = cx - fx·left/forward`，
  `v = cy - fy·up/forward`。
- 伺服：`v_x = clip(k_x(forward-look_ahead), v_min, v_max)`，
  `v_y = clip(k_y·left, ±v_max)`，`v_z = clip(k_z·up, ±v_z_max)`。
- 固定航向 `ref_yaw=0`，不做偏航跟随；底层由底座几何控制器执行位置参考。

## 安装

```bash
cd ~/uav_ws/ros2_ws/src
cp -r <本目录> .

# 仿真侧（Python ≥3.10）
uv venv ~/uav_ws/.venv --python 3.12
uv pip install --python ~/uav_ws/.venv/bin/python 'mujoco==3.12.0' numpy matplotlib

# ROS 侧（Python 3.8）
cd ~/uav_ws/ros2_ws
colcon build --packages-select uav_mujoco_sim uav_gate_center_servo
source install/setup.bash
```

`--figures` 需要 matplotlib；无头服务器先 `export MPLBACKEND=Agg`。

## 运行

```bash
# 自检（不需要 ROS）
python main.py --selftest

# 采集 55 个 oracle episode
python main.py --collect --episodes 55 --seed 0 --scene-seed 16

# 训练（默认 200 轮，早停）
python main.py --train --epochs 200 --seed 0

# 闭环/回归评测
python main.py --evaluate --episodes 50 --lateral-n 10

# 出图到 docs/img/air/uav_gate_center_servo/
python main.py --figures

# 一条命令跑完采集→训练→评测→出图
python main.py --all
```

## 话题

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/camera/depth` | `sensor_msgs/Image`（32FC1，米） | 深度输入 |
| 订阅 | `/uav/odom` | `nav_msgs/Odometry` | 姿态/位置 |
| 发布 | `/uav/cmd_vel` | `geometry_msgs/Twist` | 机体系速度，m/s |
| 发布 | `/gate_servo/status` | `std_msgs/String` | 推理次数、延迟、指令、状态 |
| 发布 | `/gate_servo/depth_view` | `sensor_msgs/Image`（mono8） | 12×16 网络输入 |

ROS 节点发布前做 `v_body = Rᵀ·a`；底座会再左乘 `R`，复原出的世界速度就是
航向系指令 `a`。不直接发布航向速度，避免姿态倾斜把指令拖到侧面。

## 文件结构

```
uav_gate_center_servo/
├── main.py
├── uav_gate_center_servo/
│   ├── cli.py / selftest.py / plotting.py
│   ├── core/
│   │   ├── depth.py       块平均、有效像素、u8 可视化
│   │   ├── geometry.py    四元数、相机偏移、标签、投影
│   │   ├── mlp.py         192→32→3 手写 MLP
│   │   ├── servo.py       比例伺服
│   │   ├── env.py         单门 MuJoCo 环境
│   │   ├── dataset.py     oracle 采集、按 episode 切分、镜像
│   │   ├── train.py       训练与回归指标
│   │   └── runner.py      闭环、延迟、横向扫描
│   └── ros/gate_servo_node.py
├── config / launch / data / test
└── README.md
```

## 结果

数据：55 个 oracle episode，4,977 帧；44/11 按 episode 切分，训练 3,970、验证 1,007。

| 项 | 值 |
|---|---|
| 验证集 distance RMSE | 0.387 m |
| forward / left / up RMSE | 0.371 / 0.092 / 0.064 m |
| 网络参数 / 权重文件 | 6,275 / 50 KB |
| 训练 | 200 轮上限，最优 159 轮，179 轮早停 |

| 闭环（50 个随机起点） | oracle | MLP |
|---|---|---|
| 通过率 | **50/50** | **48/50** |
| 平均 / 最大用时 | 4.37 / 5.50 s | 3.63 / 6.25 s |
| 推理延迟（CPU） | — | 0.048 ms 均值，0.094 ms P95 |

横向起点扫描（7 个偏移 × 10 seeds）：±1.5 m 处通过率 0.8，其余 1.0。
完整数字在 `data/results.json`，图在 `docs/img/air/uav_gate_center_servo/`。

## 验证

```bash
python main.py --selftest    # 25/26 + 1 skip（无 rclpy）
pytest -q test               # 26 passed + 1 skipped
```

自检覆盖深度降采样、标签三轴符号、相机偏移、伺服限幅、单门 pass/miss/timeout、
MLP 数值梯度/训练下降/存取、按 episode 切分、镜像、动作单位、oracle 闭环。

## 已知限制

- 单门任务，不处理门序切换；8 门全程见 `uav_depth_gate_racing_e2e`。
- 固定航向 0，不做偏航跟随；侧向误差靠平移修正。
- 深度图全零时自检/测试跳过；数据集采集直接报错，避免训练空图。
- 只验证仿真深度；真实相机的时延、缺帧和噪声未覆盖。
- ROS launch 的底座场景是 8 门，在线演示会把最近门当目标；单门指标来自离线环境。
