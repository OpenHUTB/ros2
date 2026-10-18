# 基于编码器与惯性测量单元的 MuJoCo 差速小车轮滑感知与神经网络轨迹补偿

MuJoCo Differential-Drive Slip Perception and Neural Trajectory Compensation Using Wheel Encoders and an IMU

## 模块概述 / Overview

该模块使用 MuJoCo 3.12.0 构建四轮差速小车，在 ROS 2 Humble 中提供统一的速度控制、仿真时钟、真实里程计与坐标变换接口。系统以编码器和惯性测量单元数据为主要观测量，后续可在多摩擦条件下建立轮滑感知与轨迹补偿流程。

The module uses MuJoCo 3.12.0 to model a four-wheel differential-drive robot and exposes velocity control, simulation time, ground-truth odometry, and transform interfaces in ROS 2 Humble. Wheel encoder and inertial measurement unit data form the primary observations for subsequent slip perception and trajectory compensation across multiple friction conditions.

## 系统结构 / Architecture

```text
/cmd_vel
   |
   v
差速运动学 / Differential kinematics
   |
   v
MuJoCo 四轮执行器 / Four wheel actuators
   |
   +----> /ground_truth/odom
   +----> /clock
   +----> /tf
```

功能包使用 `ament_python`，模型、配置与 launch 文件均安装到包共享目录。运行时通过 ROS 2 包索引定位资源，不依赖用户目录、固定工作空间路径或网络地址。

The package uses `ament_python`; its model, configuration, and launch files are installed into the package share directory. Runtime resource lookup uses the ROS 2 package index and does not depend on a user directory, a fixed workspace path, or a network address.

## 编码器与 IMU / Encoders and IMU

MJCF 在车体中心定义三轴加速度计和陀螺仪。节点同时读取四个轮关节的位置与角速度，并通过局部随机数生成器加入固定种子的高斯噪声。MuJoCo 内部传感器保持无噪真值，噪声只施加于发布消息，因此实验可重复且便于比较原始状态。

The MJCF defines a three-axis accelerometer and gyroscope at the chassis centre. The node also reads position and angular velocity from all four wheel joints and applies seeded Gaussian noise through a local random generator. MuJoCo sensor values remain noise-free internally; noise is added only to published messages for reproducibility and comparison with the original state.
## 环境与构建 / Environment and Build

目标环境为 Ubuntu 22.04、ROS 2 Humble、Python 3.10 和 MuJoCo 3.12.0。

The target environment is Ubuntu 22.04, ROS 2 Humble, Python 3.10, and MuJoCo 3.12.0.

```bash
python3 -m pip install -r src/ground/mujoco_diff_drive_slip_compensation/requirements.txt
source /opt/ros/humble/setup.bash
colcon build --packages-select mujoco_diff_drive_slip_compensation
source install/setup.bash
```

## 启动与检查 / Launch and Inspect

```bash
ros2 launch mujoco_diff_drive_slip_compensation main.launch.py
ros2 topic list
ros2 topic echo /ground_truth/odom
```

另一个终端可发送速度指令：

Send a velocity command from another terminal:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.4}, angular: {z: 0.25}}"
```

## 参数 / Parameters

| 参数 / Parameter | 默认值 / Default | 说明 / Description |
| --- | ---: | --- |
| `model_file` | `models/diff_drive.xml` | 包共享目录内的 MJCF 路径 / MJCF path below the package share directory |
| `wheel_radius` | `0.08` | 车轮半径，单位 m / wheel radius in metres |
| `track_width` | `0.42` | 左右轮中心距，单位 m / lateral wheel separation in metres |
| `max_wheel_speed` | `20.0` | 最大轮速，单位 rad/s / maximum wheel speed in rad/s |
| `command_timeout` | `0.5` | 自动停车超时，单位 s / automatic stop timeout in seconds |
| `publish_tf` | `true` | 是否发布坐标变换 / whether to publish transforms |
| `random_seed` | `25012` | 传感器噪声随机种子 / sensor-noise random seed |
| `friction_coefficients` | `[0.25, 0.55, 0.90]` | episode 摩擦序列 / episode friction sequence |
| `scenario_duration` | `30.0` | 单个 episode 时长，单位 s / episode duration in seconds |

## 安全边界 / Safety Boundaries

- 指令含非有限数值时拒绝更新。 / Reject commands containing non-finite values.
- 指令超时后轮速归零。 / Set wheel speeds to zero after command timeout.
- 四轮目标速度按执行器上限等比例缩放。 / Scale all wheel targets proportionally to actuator limits.
- 包资源路径经过边界检查。 / Validate package-resource paths against directory escape.

## 数据采集 / Data Collection

一条 launch 命令同时启动仿真、确定性激励指令和采集器：

One launch command starts the simulation, deterministic excitation commands, and collector:

```bash
ros2 launch mujoco_diff_drive_slip_compensation collect.launch.py \
  output_directory:=data/raw
```

场景切换时车辆复位，新的 `episode_id` 与摩擦系数一起发布。采集器在编码器消息到达时组合最近的 IMU、指令、真实里程计和场景信息。每个 episode 独立写入 CSV，并在关闭时写入 JSON 清单。

At each scenario transition, the vehicle is reset and a new `episode_id` is published with its friction coefficient. On each encoder message, the collector combines the latest IMU, command, ground-truth odometry, and scenario state. Each episode is written to its own CSV and receives a JSON manifest when closed.

### 数据字段 / Data Fields

- 时间戳、episode、数据集划分、摩擦系数。 / Timestamp, episode, dataset split, and friction coefficient.
- 线速度与角速度指令。 / Linear and angular velocity commands.
- 四轮位置与角速度。 / Four-wheel positions and angular velocities.
- 三轴加速度与角速度。 / Three-axis acceleration and angular velocity.
- 真实二维位姿与速度。 / Ground-truth planar pose and velocity.

数据集使用固定种子和 episode 标识的稳定哈希执行 70%/15%/15% 划分。划分单位是完整 episode，而不是单帧，因此相邻帧不会跨越训练、验证和测试集合。

Dataset assignment uses a stable hash of the fixed seed and episode identifier with 70%/15%/15% ratios. The assignment unit is a complete episode rather than an individual frame, so adjacent frames cannot cross train, validation, and test sets.

## 滑移感知训练 / Slip-Perception Training

训练器读取第一天生成的 CSV 与 JSON manifest，逐 episode 验证格式版本、行数、时间戳单调性和 split 一致性。窗口长度默认为 12 帧；窗口只从一个 episode 内滑动，不在 episode 或 train/validation/test 边界连接样本。归一化均值和标准差只由 train episode 计算。

The trainer reads the CSV and JSON manifests produced by the collection pipeline and validates schema version, row count, monotonic timestamps, and split consistency per episode. The default window contains 12 frames. Windows never cross episode or train/validation/test boundaries. Feature means and standard deviations are fitted only on train episodes.

当前实现采用紧凑的 temporal-window MLP：tanh 隐藏层连接可训练线性输出头，使用 Huber 损失抑制极少数大滑移比例对常规样本的影响。纵向滑移率标签由编码器轮速与 MuJoCo 真实车体速度计算：

The current model is a compact temporal-window MLP with a tanh hidden layer and a trainable linear skip head. Huber loss limits the influence of rare high-slip ratios. The longitudinal target is derived from encoder wheel speed and MuJoCo ground-truth chassis speed:

$$
\bar{\omega}_t = \frac{\omega_{LF,t}+\omega_{LR,t}+\omega_{RF,t}+\omega_{RR,t}}{4},\qquad
s_t = \operatorname{clip}\left(1-\frac{v^{truth}_t}{r\bar{\omega}_t},-0.5,0.5\right).
$$

当 `|r ω̄| < 0.15 m/s` 时将标签置零，以避免低速除法放大噪声。训练按固定随机种子打乱 train 窗口，使用 validation MAE 选择模型；测试 episode 不参与归一化或参数选择。模型以轻量 JSON 参数文件保存，不提交权重文件。

Targets are set to zero when `|r ω̄| < 0.15 m/s` to avoid amplifying noise through division near standstill. Train windows are shuffled with a fixed seed, validation MAE selects the model, and test episodes are excluded from normalization and parameter selection. The compact parameters are saved as JSON and no trained weights are committed.

### 训练流程 / Training Workflow

```text
episode CSV + manifest
        |
validate schema / split / timestamps
        |
episode-local windows + slip labels
        |
train-only normalization -> MLP fitting
        |
validation selection -> held-out test metrics
        |
slip_model.json + training_metrics.json
```

生成同格式 MuJoCo 样例数据并训练： / Generate matching MuJoCo sample episodes and train:

```bash
generate_slip_sample_data --output data/sample --episodes-per-split 4 --duration 12
train_slip_model --data data/sample --output data/models/slip-v1 --epochs 300
```

使用 ROS 2 收集数据训练： / Train from ROS 2 collected data:

```bash
train_slip_model --data data/raw --output data/models/slip-v1 --epochs 300
ros2 launch mujoco_diff_drive_slip_compensation train.launch.py \
  data_directory:=data/raw output_directory:=data/models/slip-v1
```

本机样例配置为 12 个 episode、每个 split 4 个完整 episode、12 帧窗口、24 个隐藏单元和 300 轮上限。实际提前停止于验证集不再改善时。隔离测试集含 916 个窗口，测得 `MAE=0.0272`、`RMSE=0.0440`、`F1=0.774`、`accuracy=0.969`，CPU 单窗口推理中位数为 `0.0103 ms`。该表仅代表样例测试，不代表其他摩擦和实机性能。

The measured local run used 12 episodes (four complete episodes per split), a 12-frame window, 24 hidden units, and a 300-epoch maximum with early stopping on validation MAE. Its isolated test split contained 916 windows and measured `MAE=0.0272`, `RMSE=0.0440`, `F1=0.774`, `accuracy=0.969`, and `0.0103 ms` median CPU inference per window. These measurements apply only to this sample split, not other friction settings or physical hardware.

## 在线补偿 / Online Compensation

`online_slip_compensation` 使用与离线训练相同的 12 维特征顺序，订阅 `/wheel/encoders`、`/imu/data`、`/ground_truth/odom` 和原始 `/cmd_vel`。里程计在这里仅用于时间同步与新鲜度检查，不作为模型输入。模型窗口完整且所有消息时间戳在 `sensor_timeout_seconds` 内时，节点发布 `/cmd_vel/compensated`、`/slip/estimate` 和 `/slip/status`。

The `online_slip_compensation` node keeps the offline 12-feature order and subscribes to `/wheel/encoders`, `/imu/data`, `/ground_truth/odom`, and raw `/cmd_vel`. Odometry is used for synchronization and freshness only, not as a model feature. When the window is full and all messages are fresh, it publishes `/cmd_vel/compensated`, `/slip/estimate`, and `/slip/status`.

补偿策略为：

The compensation policy is:

1. 传感器或指令缺失、时间戳回退、时间差超过阈值、输入非有限时，清空窗口并发布零速。 / Missing data, timestamp rollback, stale data, or non-finite input clears the window and publishes zero velocity.
2. `v_out = v_cmd / max(1-s, 0.5)`，再限制最大线速度比例。 / Compute `v_out = v_cmd / max(1-s, 0.5)` and then cap the linear scale.
3. 对线速度和角速度应用独立加速度限幅，避免模型输出造成突变。 / Apply independent linear and angular acceleration limits to prevent abrupt commands.

```bash
ros2 launch mujoco_diff_drive_slip_compensation compensate.launch.py \
  model_file:=data/models/slip-v1/slip_model.json
```

仿真器在 launch 组内接收 `/cmd_vel/compensated`，外部控制器仍向 `/cmd_vel` 发布原始指令；因此补偿节点不会把自己的输出再次作为输入。 / The simulator receives `/cmd_vel/compensated` inside the launch group while an external controller continues publishing raw `/cmd_vel`, so the node does not feed its own output back into its input.


## 验证命令 / Verification Commands

```bash
python3 -m pytest src/ground/mujoco_diff_drive_slip_compensation/test -q
python3 -m compileall src/ground/mujoco_diff_drive_slip_compensation
colcon test --packages-select mujoco_diff_drive_slip_compensation
colcon test-result --verbose
```
