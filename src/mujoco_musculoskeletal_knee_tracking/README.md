# mujoco_musculoskeletal_knee_tracking

单关节**膝关节**肌肉骨骼仿真与轨迹跟踪控制（MuJoCo + ROS 2 Humble）。
Musculoskeletal single-joint **knee** simulation with sine/step trajectory
tracking via PD-driven antagonistic muscles.

> 与仓库内其他肌肉骨骼模块区分：本模块聚焦**膝关节**、**基础交互与轨迹跟踪**，
> 不涉及神经网络闭环学习。The module focuses on the **knee** joint and
> **classical PD trajectory tracking**, not neural closed-loop learning.

## 1. 模型 (Model)

`model/muscle_knee.xml`（MuJoCo MJCF）：

| 部件 | 说明 |
|------|------|
| `femur`   | 股骨（固定，髋端→膝端） |
| `tibia`   | 胫骨（经 `knee` 铰链关节绕 x 轴转动） |
| `flexor`  | 屈肌（近似腘绳肌），激活→膝关节屈曲（关节角增大） |
| `extensor`| 伸肌（近似股四头肌），激活→膝关节伸展（关节角减小） |
| `knee_wrap` | 关节 wrap 圆柱：肌腱绕过关节轴，提供恒定正力矩臂 |

- 0° = 完全伸展（小腿竖直下垂）；正角 = 屈曲。
- 关节限位 `-15° ~ 90°`，建议控制轨迹 `10° ~ 70°`。
- 已本地验证：单独激活屈肌→屈曲、单独激活伸肌→伸展、无控制→0° 平衡。

## 2. 依赖 (Dependencies)

- Ubuntu 20.04 / ROS 2 Humble
- MuJoCo: `pip install mujoco`
- numpy / matplotlib（离线演示脚本用）

## 3. 构建与运行 (Build & Run)

```bash
cd ~/ros2_ws/src          # 放入你的工作空间
git clone <本模块> mujoco_musculoskeletal_knee_tracking
cd ~/ros2_ws
colcon build --packages-select mujoco_musculoskeletal_knee_tracking
source install/setup.bash

# 方式一: launch (推荐)
ros2 launch mujoco_musculoskeletal_knee_tracking main.launch.py

# 方式二: 直接运行
ros2 run mujoco_musculoskeletal_knee_tracking mujoco_musculoskeletal_knee_tracking_node
```

查看输出：

```bash
ros2 topic echo /knee/angle        # 当前关节角 [deg]
ros2 topic echo /knee/activation   # [flexor, extensor] 激活
ros2 topic echo /knee/torque       # 实际广义力矩
```

## 4. 话题接口 (Topics)

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `knee/reference` | `std_msgs/Float64` | 订阅 | 目标角度 [deg]，可外部下发 |
| `knee/angle`     | `std_msgs/Float64` | 发布 | 当前关节角 [deg] |
| `knee/activation`| `std_msgs/Float64MultiArray` | 发布 | `[flexor, extensor]` 激活 |
| `knee/torque`    | `std_msgs/Float64` | 发布 | 执行器广义力 |

外部下发目标（覆盖内置轨迹）示例：

```bash
ros2 topic pub /knee/reference std_msgs/Float64 "{data: 50.0}" --once
```

## 5. 离线演示 (Offline demo, 不依赖 ROS2)

生成轨迹跟踪 GIF / 评价图 / 指标，便于提交 PR 附运行效果：

```bash
python3 simulate_tracking.py
```

输出：`tracking.gif`（<10MB）、`tracking_eval.png`、`tracking_metrics.txt`。

默认结果（10s 正弦，bias=30° amp=40° f=0.4Hz，PD kp=30 kd=3）：
RMSE ≈ 11°（PD 正弦跟踪固有滞后所致），收敛时间 ≈ 1.96s。

## 6. 参数 (Parameters, `config/params.yaml`)

| 参数 | 默认 | 说明 |
|------|------|------|
| `sim.dt` | 0.005 | 仿真步长 [s] |
| `sim.duration` | 30.0 | 内置轨迹时长 [s] |
| `trajectory.type` | sine | `sine` \| `step` |
| `trajectory.amplitude_deg` | 40.0 | 轨迹幅值 [deg] |
| `trajectory.frequency_hz` | 0.4 | 正弦频率 [Hz] |
| `trajectory.bias_deg` | 30.0 | 轨迹偏置 [deg] |
| `ctrl.kp` | 30.0 | 比例增益 [N·m/rad] |
| `ctrl.kd` | 3.0 | 微分增益 [N·m·s/rad] |
| `ctrl.max_torque` | 25.0 | 肌肉等效最大力矩 [N·m] |
| `ctrl.limit_deg` | 90.0 | 屈曲限位 [deg] |

## 7. 单元测试

```bash
colcon test --packages-select mujoco_musculoskeletal_knee_tracking
```

覆盖：激活边界、拮抗分配、限位保护、非法输入（NaN/Inf）安全停止。

## 8. 提交注意事项 (Submitting a PR)

- 文档放入仓库 `docs/` 与 `docs/musculoskeletal/`；源码放入 `src/`。
- 入口 `main.*`，必须支持 `launch`。
- PR 附运行效果图（`tracking.gif` / `tracking_eval.png`），并写明运行环境与步骤。
- 本模块使用大模型辅助开发，已在提交中声明并对内容负责。
