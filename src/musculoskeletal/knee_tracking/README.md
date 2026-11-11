# 膝关节肌肉骨骼仿真与轨迹跟踪（MuJoCo + ROS 2）

单关节**膝关节**：两条拮抗肌（屈肌 / 伸肌）经恒定力臂驱动胫骨，PD 跟踪正弦 /
阶跃轨迹，并发布关节角、肌肉激活与实际力矩。补的是
[`docs/musculoskeletal/knee_tracking.md`](../../docs/musculoskeletal/knee_tracking.md)
里描述、但此前没有 `src/` 实现的模块。

- 包名：`mujoco_musculoskeletal_knee_tracking`（与文档一致）
- 构建：`ament_cmake`
- 依赖：`mujoco`（pip）、`rclpy`、`std_msgs`、`python3-yaml`

## 1. 模型

`assets/knee.xml`：

| 项目 | 取值 |
| --- | --- |
| 关节 | 铰链，绕 +y，限位 −15°…90° |
| 0° 定义 | 小腿竖直下垂、完全伸展；正角 = 屈曲（向后抬起） |
| 胫骨质量 / 质心 | 3.431 kg / 距膝 0.15 m → `m·g·l = 5.05 N·m` |
| 关节阻尼 / 电枢 | 0.4 N·m·s/rad / 0.001 |
| 肌肉 | 2 条（屈肌、伸肌），`F_max = 500 N`，力臂 `r = 0.035 m` |
| 最大肌肉力矩 | `500 × 0.035 = 17.5 N·m` |
| 积分器 / 步长 | `implicitfast` / 5 ms（写入 `model.opt.timestep`） |

两条肌肉用 `motor` 执行器 + 恒定 `gear = ±0.035` 表示（等价于文档里的 wrap 圆柱
提供恒定力矩臂），`ctrl` 直接是肌力 [N]；Hill 侧的激活动力学在 Python 里算：

```
τ = kp·e − kd·θ̇,   e = θ_ref − θ
u_flex = max(τ, 0)/17.5,  u_ext = max(−τ, 0)/17.5
ȧ = (u − a)/τ_act (u > a),  (u − a)/τ_deact (u ≤ a)
F_flex = a_flex·F_max,  F_ext = a_ext·F_max
```

## 2. 配置（`config/params.yaml`）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `sim.dt` | 0.005 | 仿真步长 [s] |
| `sim.duration` | 30.0 | 内置轨迹时长 [s] |
| `trajectory.type` | sine | `sine` / `step` |
| `trajectory.amplitude_deg` | 40.0 | 幅值 |
| `trajectory.frequency_hz` | 0.4 | 频率 |
| `trajectory.bias_deg` | 30.0 | 偏置 |
| `ctrl.kp` / `ctrl.kd` | 30 / 3 | PD 增益 |
| `ctrl.max_torque` | 17.5 | 肌肉等效最大力矩 |
| `ctrl.limit_deg` / `limit_low_deg` | 90 / −15 | 关节限位 |
| `muscle.tau_act` / `tau_deact` | 0.02 / 0.05 | 激活 / 失活时间常数 |

## 3. 运行

```bash
# 依赖
python3 -m pip install -r requirements.txt

cd ~/ros2_ws
colcon build --packages-select mujoco_musculoskeletal_knee_tracking
source install/setup.bash

# 节点
ros2 run mujoco_musculoskeletal_knee_tracking mujoco_musculoskeletal_knee_tracking_node
ros2 launch mujoco_musculoskeletal_knee_tracking main.launch.py duration:=10.0 exit_after_duration:=true

# 话题
ros2 topic echo /knee/angle
ros2 topic echo /knee/activation
ros2 topic echo /knee/torque
ros2 topic pub /knee/reference std_msgs/Float64 "{data: 50.0}" --once
```

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/knee/angle` | `std_msgs/Float64` | 关节角 [deg] |
| `/knee/activation` | `std_msgs/Float64MultiArray` | `[屈肌, 伸肌]` 激活 |
| `/knee/torque` | `std_msgs/Float64` | 肌肉净力矩 [N·m] |
| `/knee/reference` | `std_msgs/Float64` | 外部目标 [deg]，收到后切外部跟踪 |

收到外部目标后不再受 `duration` 限制；`exit_after_duration:=true` 时内置轨迹跑完
节点自动退出（便于 CI/无人值守）。

离线（不需要 ROS）：

```bash
python3 simulate_tracking.py            # CSV + 指标（有 matplotlib 时出 PNG）
python3 simulate_tracking.py --no-render
python3 validate_model.py               # 模型自检
```

## 4. 实测结果（WSL2 Ubuntu 24.04 + MuJoCo 4.0.0）

| 指标 | 本文档实现 | 文档记录 |
| --- | --- | --- |
| 正弦 RMSE | **8.57°** | 8.65° |
| 正弦最大绝对误差 | 15.04° | 13.96° |
| 阶跃超调 | **0.00%** | 0.0% |
| 阶跃调节时间（±2%） | 未进入 | 未进入 |
| 阶跃稳态误差 | **−6.62°** | −6.62° |
| ±5° 首次进入 | 未进入（峰值 44.06°） | 0.22 s |

稳态下垂完全由重力矩解释：`kp·e_ss = mgl·sin θ` → `e_ss = −6.62°`，与文档一致。
正弦残差主要来自肌肉激活/失活滞后（0.02/0.05 s），不是 PD 增益不够。

**与文档的已知差异**：±5° 首次进入时间。文档的峰值略高于 45°，本实现的峰值
44.06°（相差不到 1°），因此按“进入并保持”的口径判为未进入。差异来自胫骨惯量
与阻尼的取值，文档未给出这两项。

## 5. 测试

```bash
python3 -m pytest -q test/          # 21 个用例
colcon test --packages-select mujoco_musculoskeletal_knee_tracking
```

覆盖：MJCF 结构（nu=2、单关节、限位、步长）、重力矩幅值、PD 符号与饱和、
激活一阶动力学（上升/下降时间常数、限幅）、正弦/阶跃指标定义、与文档数量级一致。

## 6. 文件结构

```
mujoco_musculoskeletal_knee_tracking/
├── assets/knee.xml                     # MJCF：股骨 + 胫骨 + 两条肌肉
├── config/params.yaml                  # 全部参数
├── launch/main.launch.py
├── mujoco_musculoskeletal_knee_tracking/
│   ├── config.py                       # 参数加载与校验
│   ├── model.py                        # MuJoCo 仿真 + Hill 肌肉 + PD
│   ├── simulation.py                   # 轨迹、指标
│   └── node.py                         # rclpy 节点
├── simulate_tracking.py                # 离线演示（CSV/PNG/指标）
├── validate_model.py                   # 模型自检
└── test/
```

## 7. 已知限制

- 肌肉力只按 `a·F_max` 计，力-长度、力-速度因子取 1（与文档默认参数一致，
  但不等同于完整 Hill 模型）；
- 恒定力臂用 `gear` 表示，没有建 wrap 圆柱的几何接触；
- 只做矢状面单关节，不含髌骨、滚动-滑动与多关节耦合；
- 外部目标通过话题直接给角度，没有做轨迹插值，阶跃过大会触发限位保护。
