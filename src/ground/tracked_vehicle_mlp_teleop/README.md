# tracked_vehicle_mlp_teleop

ROS 1 Noetic + Gazebo 11 的**履带式地面车辆物理仿真**模块：键盘遥操作、PI 基线、
以及一个由行为克隆训练的 PyTorch MLP 打滑/坡度补偿控制器。模块在平地、10° 斜坡、
低摩擦三种场景中完成多场景测试，并输出可复现的 CSV 指标、图表和 GIF 演示。

> 对应任务：任务 1（车辆物理仿真 + 键盘控制），并额外加入神经网络控制算法以满足
> “感知/规划/控制算法必须使用神经网络”的要求。

## 1. 运行环境（实测）

| 项目 | 版本 |
|---|---|
| 操作系统 | Ubuntu 20.04.6 LTS |
| ROS | Noetic Ninjemys |
| Gazebo | Gazebo 11.10.2 |
| Python | 3.8 |
| PyTorch | 2.4.1（CPU 即可） |
| NumPy | 1.24 |
| Matplotlib | 3.7（仅评估绘图） |

无需 CARLA、AirSim、HoloOcean 或 GPU；整个模块只包含文本模型、世界文件和
< 100 KB 的 MLP 权重。

## 2. 目录结构

```
tracked_vehicle_mlp_teleop/
├── launch/
│   ├── main.launch              # 统一入口：键盘 / 采集 / 评估
│   ├── main_flat.launch         # 平地键盘演示
│   ├── main_slope.launch        # 斜坡键盘演示
│   ├── main_low_friction.launch # 低摩擦键盘演示
│   ├── collect.launch           # 无界面数据采集
│   └── eval.launch              # 无界面多控制器评估
├── urdf/
│   └── tracked_vehicle.xacro    # 履带车模型 + 差速驱动/IMU 插件
├── worlds/
│   ├── flat.world               # 平地，mu=1.0
│   ├── slope.world              # 10° 斜坡 + 顶部平台
│   └── low_friction.world       # 低摩擦地面，mu=0.12
├── config/params.yaml           # 车辆、PI、MLP、采集、评估参数
├── scripts/
│   ├── main.py                  # 主入口：键盘/采集/评估 + MLP 推理
│   ├── model.py                 # MLP、归一化、PI 专家
│   ├── train_mlp.py             # 行为克隆训练
│   ├── evaluate.py              # 汇总指标 + 绘图
│   ├── collect_all.sh           # 三场景采集 + 训练
│   ├── run_benchmark.sh         # 三场景 RAW/PI/MLP 评估
│   └── test_offline.py          # 不依赖 ROS 的离线自检
├── models/
│   ├── mlp_policy.pt            # 约 23 KB
│   ├── norm_stats.npz           # 约 1 KB
│   └── model_config.json
├── data/                        # 采集 CSV、评估 CSV、汇总表（运行采集/评估脚本后生成，不入库）
└── （演示 GIF 与截图位于 docs/img/ground/tracked_vehicle_mlp_teleop/）
```

## 3. 编译

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps tracked_vehicle_mlp_teleop
source devel/setup.bash
```

## 4. 键盘控制演示

```bash
roslaunch tracked_vehicle_mlp_teleop main.launch world:=flat gui:=true
```

终端按键：

| 按键 | 功能 |
|---|---|
| `W` / `S` | 前进 / 后退（每次 0.1 m/s） |
| `A` / `D` | 左转 / 右转（每次 0.15 rad/s） |
| `Q` / 空格 | 停车 |
| `1` / `2` / `3` | 切换 RAW / PI / MLP 控制器 |
| `N` | 循环切换控制器 |
| `R` | 复位 Gazebo 世界 |
| `H` | 显示按键帮助 |
| `ESC` | 退出 |

斜坡与低摩擦场景：

```bash
roslaunch tracked_vehicle_mlp_teleop main_slope.launch
roslaunch tracked_vehicle_mlp_teleop main_low_friction.launch
```

> 若从 `roslaunch` 启动时终端没有按键响应，请确认是在真实终端中运行；
> `main.py` 会直接读取 `/dev/tty`，不依赖 `roslaunch` 的 stdin 转发。

## 5. 采集、训练与评估（一键复现）

```bash
# 1) 三个场景各采集 60 s PI 专家数据，并训练 MLP
bash $(rospack find tracked_vehicle_mlp_teleop)/scripts/collect_all.sh

# 2) 三个场景依次评估 RAW / PI / MLP，生成汇总表与图
bash $(rospack find tracked_vehicle_mlp_teleop)/scripts/run_benchmark.sh
```

也可以单独运行：

```bash
# 采集
roslaunch tracked_vehicle_mlp_teleop collect.launch \
  world:=flat duration:=60 \
  output_csv:=$(rospack find tracked_vehicle_mlp_teleop)/data/collect_flat.csv

# 训练
python3 $(rospack find tracked_vehicle_mlp_teleop)/scripts/train_mlp.py \
  --csv .../data/collect_flat.csv .../data/collect_slope.csv .../data/collect_low_friction.csv

# 评估（无界面）
roslaunch tracked_vehicle_mlp_teleop eval.launch world:=flat \
  output_csv:=$(rospack find tracked_vehicle_mlp_teleop)/data/eval_flat.csv \
  summary_csv:=$(rospack find tracked_vehicle_mlp_teleop)/data/summary_flat.csv
```

> 如果机器上已有其他 Gazebo/ROS 实例，请设置独立端口，避免互相干扰：
> `export ROS_MASTER_URI=http://127.0.0.1:11411`
> `export GAZEBO_MASTER_URI=http://127.0.0.1:11355`

## 6. 话题与坐标系

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/odom` | `nav_msgs/Odometry` | 世界坐标系实际速度（`odometrySource=world`） |
| 订阅 | `/imu` | `sensor_msgs/Imu` | 姿态、俯仰/横滚、角速度 |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` | 左右履带差速指令 |
| 发布 | `/joint_states` | `sensor_msgs/JointState` | 轮关节状态 |
| TF | `odom -> base_link` | — | 差速插件发布 |

## 7. 神经网络控制

- 输入 11 维：`v_target, w_target, v_meas, w_meas, e_v, e_w, int_v, int_w, pitch,
  roll, yaw_rate`。
- 结构：`11 -> 64 -> 64 -> 2`，ReLU，输出归一化 `(v_cmd, w_cmd)`。
- 训练：PI 专家在三种场景中驱动车辆，记录状态、误差、积分项与专家指令；
  MLP 以 MSE 行为克隆专家控制律，并保存特征标准化参数。
- 部署：`main.py` 以 20 Hz 读取 `/odom` 与 `/imu`，MLP 直接输出差速指令；
  可随时用 `1/2/3` 与 PI、纯运动学基线对比。

训练细节、算法流程和源码解析见
[docs/ground/tracked_vehicle_mlp_teleop.md](../../../docs/ground/tracked_vehicle_mlp_teleop.md)。

## 8. 多场景与性能指标

评估剖面：起步 0.5 s → 直行 3 s → 右转 3 s → 左转 3 s → 直行 2 s；
每个控制器前通过 `/gazebo/reset_world` 复位，保证对比公平。

| 指标 | 定义 |
|---|---|
| `rmse_v` / `rmse_w` | 目标速度与实际速度的均方根误差 |
| `mean_abs_slip` | 实际线速度与下发指令之差的平均绝对值，反映打滑 |
| `distance_m` | 评估期间实际行驶距离 |
| `max_roll_deg` / `max_pitch_deg` | 最大横滚/俯仰角，反映稳定性 |
| `latency_mean_ms` / `latency_p95_ms` | 控制器单次推理耗时 |
| `success` | 未侧翻（横滚 < 34°、俯仰 < 52°）且完成评估剖面 |

实测结果（运行评估后生成 `data/summary_table.md`，指标图见 `docs/img/ground/tracked_vehicle_mlp_teleop/metrics.png`）：

| 场景 | 控制器 | RMSE_v (m/s) | RMSE_w (rad/s) | 平均打滑 (m/s) | 距离 (m) | 最大侧倾 (°) | 推理耗时 (ms) | 成功 |
|---|---|---|---|---|---|---|---|---|
| 平地 | RAW | 0.049 | 0.093 | 0.009 | 4.54 | 0.0 | — | 是 |
| 平地 | PI | 0.067 | 0.426 | 0.083 | 4.57 | 0.0 | — | 是 |
| 平地 | MLP | 0.056 | 0.460 | 0.059 | 4.58 | 0.0 | 0.54 | 是 |
| 10°斜坡 | RAW | 0.072 | 0.107 | 0.027 | 4.34 | 7.2 | — | 是 |
| 10°斜坡 | PI | 0.152 | 0.418 | 0.218 | 4.55 | 7.7 | — | 是 |
| 10°斜坡 | MLP | 0.084 | 0.382 | 0.097 | 4.44 | 7.9 | 0.49 | 是 |
| 低摩擦 | RAW | 0.078 | 0.141 | 0.021 | 4.42 | 0.0 | — | 是 |
| 低摩擦 | PI | 0.087 | 0.144 | 0.057 | 4.57 | 0.0 | — | 是 |
| 低摩擦 | MLP | 0.088 | 0.152 | 0.054 | 4.57 | 0.0 | 0.49 | 是 |

关键结论：10° 斜坡上 MLP 的 `rmse_v=0.084`、`mean_abs_slip=0.097`，
均显著优于 PI（`0.152`、`0.218`）；MLP 单次推理约 0.5 ms，实时性充足。

## 9. 演示素材

- `docs/img/ground/tracked_vehicle_mlp_teleop/demo.gif`：三场景键盘/MLP 演示（10–20 MB）。
- `docs/img/ground/tracked_vehicle_mlp_teleop/flat.png`、`slope.png`、`low_friction.png`：场景截图。
- `docs/img/ground/tracked_vehicle_mlp_teleop/metrics.png`：RAW / PI / MLP 指标对比图。

## 10. 已知限制

- 履带外观为视觉件，实际动力学由左右驱动轮 + 前后万向轮近似，属于简化履带模型；
  打滑通过轮地摩擦与低摩擦地面产生，并非完整履带链节动力学。
- MLP 由 PI 专家行为克隆得到，优势主要在坡度/低摩擦场景的补偿一致性；
  在分布外场景（例如极端摩擦系数）需要重新采集数据训练。
- 评估只使用固定速度剖面，未加入外部扰动或动态障碍物。

## 11. 许可证

MIT
