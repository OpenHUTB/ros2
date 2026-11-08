# mujoco_musculoskeletal_knee_tracking

单关节**膝关节**肌肉骨骼仿真与轨迹跟踪控制（MuJoCo + ROS 2 Humble）。
Musculoskeletal single-joint **knee** simulation with sine/step trajectory
tracking via PD-driven antagonistic muscles.

> 与仓库内其他肌肉骨骼模块区分：本模块聚焦**膝关节**、**基础交互与轨迹跟踪**，
> 不涉及神经网络闭环学习。

## 1. 模型 (Model)

`mujoco_musculoskeletal_knee_tracking/model/muscle_knee.xml`（MuJoCo MJCF）：

| 部件 | 说明 |
|------|------|
| `femur`   | 股骨（固定，髋端→膝端） |
| `tibia`   | 胫骨（经 `knee` 铰链关节绕 x 轴转动） |
| `flexor`  | 屈肌（近似腘绳肌），激活→膝关节屈曲（关节角增大） |
| `extensor`| 伸肌（近似股四头肌），激活→膝关节伸展（关节角减小） |
| `knee_wrap` | 关节 wrap 圆柱：肌腱绕过关节轴，提供恒定正力矩臂 |

- 0° = 完全伸展（小腿竖直下垂）；正角 = 屈曲。
- 关节限位 `-15° ~ 90°`，建议控制轨迹 `10° ~ 70°`。

## 2. 运行环境 (Environment)

| 项 | 版本 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| Python | 3.10 |
| MuJoCo | 3.16.0（`pip install mujoco`） |
| numpy / matplotlib / Pillow | 见 `requirements.txt` |

```
python3 -m pip install -r requirements.txt
```

## 3. 构建与运行 (Build & Run)

```
cd ~/ros2_ws/src
git clone <本模块> mujoco_musculoskeletal_knee_tracking
cd ~/ros2_ws
colcon build --packages-select mujoco_musculoskeletal_knee_tracking
source install/setup.bash

ros2 launch mujoco_musculoskeletal_knee_tracking main.launch.py
# 或
ros2 run mujoco_musculoskeletal_knee_tracking mujoco_musculoskeletal_knee_tracking_node
```

查看输出：

```
ros2 topic echo /knee/angle
ros2 topic echo /knee/activation
ros2 topic echo /knee/torque
```

## 4. 话题接口 (Topics)

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `knee/reference` | `std_msgs/Float64` | 订阅 | 目标角度 [deg]，可外部下发 |
| `knee/angle`     | `std_msgs/Float64` | 发布 | 当前关节角 [deg] |
| `knee/activation`| `std_msgs/Float64MultiArray` | 发布 | `[flexor, extensor]` 激活 |
| `knee/torque`    | `std_msgs/Float64` | 发布 | 执行器广义力 |

```
ros2 topic pub /knee/reference std_msgs/Float64 "{data: 50.0}" --once
```

## 5. 离线演示 (Offline demo, 不依赖 ROS 2)

```
python3 simulate_tracking.py               # 生成 GIF + 曲线 + 指标
python3 simulate_tracking.py --no-render   # 无图形/远程环境：只出曲线与指标
```

输出（默认 `results/`）：`tracking.gif`（< 5 MB）、`tracking_eval.png`、`tracking_step.png`、`tracking_metrics.txt`。

默认结果（10 s 正弦，bias=30° amp=40° f=0.4 Hz，PD kp=30 kd=3）：
正弦段 RMSE ≈ 8.65°、最大误差 13.96°（参考持续变化，不适用超调/调节时间）。
阶跃段（20°→50°）超调 0 %、未进入 ±2% 带、稳态下垂 −6.62°，主因是重力矩未做前馈补偿。

## 6. 参数 (Parameters, `config/params.yaml`)

| 参数 | 默认 | 说明 |
|------|------|------|
| `sim.dt` | 0.005 | 仿真步长 [s]，会写入 `model.opt.timestep` |
| `sim.duration` | 30.0 | 内置轨迹时长 [s] |
| `sim.exit_after_duration` | false | true：内置轨迹跑完后节点自动退出 |
| `trajectory.type` | sine | `sine` \| `step` |
| `trajectory.amplitude_deg` | 40.0 | 轨迹幅值 [deg] |
| `trajectory.frequency_hz` | 0.4 | 正弦频率 [Hz] |
| `trajectory.bias_deg` | 30.0 | 轨迹偏置 [deg] |
| `ctrl.kp` | 30.0 | 比例增益 [N·m/rad] |
| `ctrl.kd` | 3.0 | 微分增益 [N·m·s/rad] |
| `ctrl.max_torque` | 17.5 | 肌肉等效最大力矩 [N·m]，= 单肌 500 N × wrap 半径 0.035 m |
| `ctrl.limit_deg` | 90.0 | 屈曲上限 [deg] |
| `ctrl.limit_low_deg` | -15.0 | 伸展下限 [deg] |

内置轨迹跑满 `sim.duration` 后节点执行安全停止；收到 `/knee/reference` 后切换到外部跟踪模式，不再受时长限制。

## 7. 单元测试

```
colcon test --packages-select mujoco_musculoskeletal_knee_tracking
```

覆盖：激活边界、拮抗分配、限位保护、非法输入（NaN/Inf）安全停止。

## 8. 本地验证

```
python3 validate_model.py               # 含渲染
python3 validate_model.py --no-render   # 无图形环境
```

> 本模块开发过程中使用了大模型辅助，并对提交内容负责。
