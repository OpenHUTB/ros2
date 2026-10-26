# 膝关节肌肉骨骼仿真与轨迹跟踪控制 (mujoco_musculoskeletal_knee_tracking)

> 模块归属：**肌肉骨骼人 / 下肢** · 功能：**控制**
> 运行环境：Ubuntu 20.04 + ROS 2 Humble + MuJoCo

本模块使用 MuJoCo 建立单关节**膝关节**肌肉骨骼模型，以**屈肌（腘绳肌近似）/ 伸肌（股四头肌近似）** 两条拮抗肌肉驱动胫骨，通过 PD 控制器实现正弦 / 阶跃轨迹跟踪，并发布关节角、肌肉激活与实际力矩。适合用于理解"肌肉如何通过肌腱驱动机器人关节、PD 控制器如何跟踪轨迹"。

## 1. 原理 (Theory)

### 1.1 肌肉骨骼模型
- **股骨（femur）**：固定于髋端，向下延伸；**胫骨（tibia）** 通过 `knee` 铰链关节（绕 x 轴）连接，可屈伸。
- 两条拮抗肌肉通过**肌腱（tendon）** 驱动关节：
  - 屈肌（flexor）：激活时产生**屈曲**（关节角增大）；
  - 伸肌（extensor）：激活时产生**伸展**（关节角减小）。
- 肌腱绕过关节处的 **wrap 圆柱（knee_wrap）**，提供**恒定正力矩臂**，避免肌腱在屈曲大角度时长度趋零引发的非物理被动力。

### 1.2 坐标约定
- `0°` = 完全伸展（小腿竖直下垂，自然平衡）；
- 正角 = 屈曲；关节限位 `-15° ~ 90°`，建议控制轨迹 `10° ~ 70°`。

### 1.3 PD 控制器 → 拮抗肌肉激活
记误差 `e = θ_desired − θ_actual`，关节角速度 `q̇`：

```
τ = kp·e − kd·q̇
```

所需力矩 `τ` 归一化到肌肉激活（`max_torque` 为肌肉等效最大力矩）：

```
若 τ ≥ 0 (需屈曲):  flexor = clamp(τ / max_torque),  extensor = 0
若 τ < 0 (需伸展):  extensor = clamp(−τ / max_torque), flexor = 0
```

另含两类安全保护：**数值检查**（非法输入 → 激活归零）与**屈曲限位保护**（到限位且仍需屈曲 → 停屈肌）。

## 2. 接口 (Interfaces)

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `knee/reference` | `std_msgs/Float64` | 订阅 | 目标角度 [deg]，外部下发可覆盖内置轨迹 |
| `knee/angle`     | `std_msgs/Float64` | 发布 | 当前关节角 [deg] |
| `knee/activation`| `std_msgs/Float64MultiArray` | 发布 | `[flexor, extensor]` 激活 |
| `knee/torque`    | `std_msgs/Float64` | 发布 | 执行器广义力 [N·m] |

## 3. 配置 (Configuration)

见 `config/params.yaml`：

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

## 4. 构建 (Build)

```bash
cd ~/ros2_ws/src
git clone <模块仓库> mujoco_musculoskeletal_knee_tracking
cd ~/ros2_ws
colcon build --packages-select mujoco_musculoskeletal_knee_tracking
source install/setup.bash
```

## 5. 运行 (Run)

```bash
# 推荐: launch
ros2 launch mujoco_musculoskeletal_knee_tracking main.launch.py

# 或: 直接运行
ros2 run mujoco_musculoskeletal_knee_tracking mujoco_musculoskeletal_knee_tracking_node
```

查看输出：

```bash
ros2 topic echo /knee/angle
ros2 topic echo /knee/activation
ros2 topic echo /knee/torque
```

外部下发目标角度（覆盖内置正弦轨迹）：

```bash
ros2 topic pub /knee/reference std_msgs/Float64 "{data: 50.0}" --once
```

## 6. 离线演示 (Offline demo)

不依赖 ROS2，直接生成运行效果 GIF / 评价图 / 指标：

```bash
python3 simulate_tracking.py
```

输出：`tracking.gif`（<10MB）、`tracking_eval.png`、`tracking_metrics.txt`。

## 7. 数据与评价 (Data & Metrics)

`tracking_metrics.txt` 报告：

- **RMSE**：角度误差均方根（目标 vs 实际）；
- **超调量**：正方向超调；
- **收敛时间**：误差首次进入 ±3° 的时间。

默认 10s 正弦轨迹（bias=30° amp=40° f=0.4Hz，PD kp=30 kd=3）结果：
RMSE ≈ 11°，收敛 ≈ 1.96s。PD 对正弦跟踪存在固有相位滞后，故 RMSE 非零属正常。

## 8. 故障处理 (Troubleshooting)

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 关节不运动 | `mujoco` 未安装 | `pip install mujoco` |
| 跟踪幅值明显不足 | `kp` 过小或 `max_torque` 过大 | 增大 `kp`，确认 `max_torque` 约等于肌肉实际最大力矩（本模型 ≈25 N·m） |
| 高频振荡 | `kd` 不足 | 增大 `kd` |
| 触到限位剧烈冲击 | 轨迹幅值过大 | 控制轨迹保持在 `10°~70°` 内 |
| 模型无法加载（中文路径） | 文件路径含特殊字符 | 用 `from_xml_string` 读取（本模块已内置），或使用纯英文路径 |

## 9. 附：本地验证 (Local validation)

```bash
python3 validate_model.py
```

验证：模型可加载、单独激活屈肌→屈曲、单独激活伸肌→伸展、无控制→0° 平衡、并输出 `model_render.png`。

> 本模块开发过程中使用了大模型辅助（代码生成与调试），并对提交内容负责。
