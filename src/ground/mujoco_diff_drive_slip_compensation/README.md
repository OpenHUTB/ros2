# MuJoCo 差速小车轮滑感知与轨迹补偿

MuJoCo Differential-Drive Slip Perception and Trajectory Compensation

本 ROS 2 功能包提供一个四轮差速小车的 MuJoCo 仿真基础，并发布带可复现噪声的编码器和惯性测量单元数据。多摩擦采集入口按完整 episode 轮换场景、复位车辆、执行确定性激励指令，并把同步观测写入按 episode 隔离的数据文件。

This ROS 2 package provides a MuJoCo simulation foundation for a four-wheel differential-drive robot and publishes encoder and inertial measurement unit data with reproducible noise. The multi-friction collection entry point rotates scenarios by complete episode, resets the vehicle, executes a deterministic excitation profile, and writes synchronized observations to episode-isolated files.

## 环境 / Environment

| 组件 / Component | 版本 / Version |
| --- | --- |
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| Python | 3.10 |
| MuJoCo | 3.12.0 |
| 数组计算 / Array math | NumPy 1.24+，训练和推理均运行于 CPU / NumPy 1.24+, CPU-only training and inference |

## 目录 / Layout

```text
mujoco_diff_drive_slip_compensation/
├── config/simulation.yaml
├── launch/main.launch.py
├── models/diff_drive.xml
├── mujoco_diff_drive_slip_compensation/
│   ├── kinematics.py
│   ├── main.py
│   ├── collector.py
│   ├── command_profile.py
│   ├── dataset.py
│   ├── generate_sample_dataset.py
│   ├── resources.py
│   ├── scenarios.py
│   ├── sensors.py
│   ├── simulator.py
│   └── slip_model.py
├── test/
├── package.xml
├── requirements.txt
├── setup.cfg
└── setup.py
```

## 安装与构建 / Install and Build

在 ROS 2 工作空间根目录执行：

Run the following commands from the ROS 2 workspace root:

```bash
python3 -m pip install -r src/ground/mujoco_diff_drive_slip_compensation/requirements.txt
source /opt/ros/humble/setup.bash
colcon build --packages-select mujoco_diff_drive_slip_compensation
source install/setup.bash
```

## 运行 / Run

```bash
ros2 launch mujoco_diff_drive_slip_compensation main.launch.py
```

发送 0.4 m/s 的前进指令：

Send a 0.4 m/s forward command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.4}, angular: {z: 0.0}}"
```

## 接口 / Interfaces

| 方向 / Direction | 名称 / Name | 类型 / Type | 说明 / Description |
| --- | --- | --- | --- |
| 订阅 / Subscribe | `/cmd_vel` | `geometry_msgs/msg/Twist` | 车体速度指令 / chassis velocity command |
| 发布 / Publish | `/ground_truth/odom` | `nav_msgs/msg/Odometry` | MuJoCo 真实状态 / MuJoCo ground truth |
| 发布 / Publish | `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时钟 / simulation clock |
| 发布 / Publish | `/tf` | `tf2_msgs/msg/TFMessage` | `odom` 到 `base_link` / `odom` to `base_link` |
| 发布 / Publish | `/wheel/encoders` | `sensor_msgs/msg/JointState` | 四轮位置与速度 / four-wheel positions and velocities |
| 发布 / Publish | `/imu/data` | `sensor_msgs/msg/Imu` | 三轴加速度与角速度 / three-axis acceleration and angular velocity |
| 发布 / Publish | `/scenario/friction` | `std_msgs/msg/Float64` | 当前滑动摩擦系数 / active sliding-friction coefficient |
| 发布 / Publish | `/scenario/episode_id` | `std_msgs/msg/String` | 当前完整 episode 标识 / active complete-episode identifier |

速度指令超过 `command_timeout` 未更新时，节点将四轮目标速度清零。非有限数值和越出包资源目录的模型路径会被拒绝。

When no velocity command arrives within `command_timeout`, the node sets all wheel targets to zero. Non-finite commands and model paths that escape the package share directory are rejected.

## 测试 / Test

```bash
python3 -m pytest src/ground/mujoco_diff_drive_slip_compensation/test -q
python3 -m compileall src/ground/mujoco_diff_drive_slip_compensation
```

ROS 2 环境中的完整验收还应包含 `colcon test` 与启动后的话题检查。

Full acceptance in a ROS 2 environment also includes `colcon test` and topic inspection after launch.

## 多摩擦数据采集 / Multi-friction Data Collection

默认场景使用 `0.25`、`0.55`、`0.90` 三个滑动摩擦系数，每个 episode 持续 30 秒。固定随机种子 `25012` 控制传感器噪声和数据集划分。

The default scenarios use sliding-friction coefficients `0.25`, `0.55`, and `0.90`, with each episode lasting 30 seconds. The fixed random seed `25012` controls sensor noise and dataset assignment.

```bash
ros2 launch mujoco_diff_drive_slip_compensation collect.launch.py \
  output_directory:=data/raw
```

每个 episode 生成同名 `.csv` 和 `.json`。CSV 保存指令、四轮编码器、IMU、摩擦系数和真实位姿；JSON 记录数据格式版本、划分、行数、随机种子、Python、MuJoCo 和功能包版本。`data/` 默认被 Git 忽略。

Each episode produces matching `.csv` and `.json` files. The CSV stores commands, four-wheel encoders, IMU measurements, friction, and ground-truth pose. The JSON records schema version, split, row count, random seed, and Python, MuJoCo, and package versions. `data/` is ignored by Git by default.

训练、验证、测试划分由 `seed:episode_id` 的 SHA-256 哈希确定，默认比例为 70%/15%/15%。一个 episode 的全部相邻帧始终属于同一划分。

Train, validation, and test assignment uses the SHA-256 hash of `seed:episode_id`, with default ratios of 70%/15%/15%. Every adjacent frame from one episode always remains in the same split.

## 滑移训练 / Slip Training

Use the collected ROS 2 episode directory directly. The sample generator creates the same CSV/manifest schema through the MuJoCo model and is useful when collecting through ROS 2 is not available:

直接使用 ROS 2 采集目录训练；样例生成器通过同一 MuJoCo 模型生成相同 CSV/manifest 格式的数据：

```bash
generate_slip_sample_data --output data/sample --episodes-per-split 4 --duration 12
train_slip_model --data data/raw --output data/models/slip-v1 --epochs 300
# For generated samples, replace data/raw with data/sample.
```

The temporal-window MLP uses four wheel rates, left/right wheel-rate aggregates, the velocity command, and six IMU axes. For the terminal frame of each window, the supervised longitudinal slip target is:

窗口 MLP 输入四轮角速度、左右轮聚合量、速度指令和六轴 IMU；每个时间窗口末帧的纵向滑移率标签为：

$$
\hat{s}_t = \operatorname{clip}\left(1 - \frac{v^{truth}_t}{r\,\bar{\omega}_t}, -0.5, 0.5\right),\qquad
\bar{\omega}_t = \frac{1}{4}\sum_{i=1}^{4}\omega_{i,t}.
$$

Samples with encoder-derived linear speed below `0.15 m/s` receive a zero target because the ratio is ill-conditioned near standstill. Windows are constructed independently inside each manifest-defined episode. Normalization is fitted only on train episodes; validation selects parameters and the held-out test split is used once for final MAE, RMSE, slip F1/accuracy, and median inference latency. The selected small model and metric JSON are written under `data/models/` and are not package source files.

低于 `0.15 m/s` 的编码器线速度样本标为零，因为静止附近比值条件数较差。时间窗口只在单个 manifest episode 内构造；归一化仅拟合训练 episode，验证集用于参数选择，隔离测试集用于最终 MAE、RMSE、轮滑 F1/准确率和推理中位耗时。模型与指标 JSON 写入 `data/models/`，不属于功能包源码。

The reproducible 12-episode sample run with seed `25012`, 12-frame windows, 24 hidden units, and 300 epochs measured `MAE 0.0272`, `RMSE 0.0440`, `F1 0.774`, `accuracy 0.969`, and `0.0103 ms` median inference per window on the local CPU. These values describe that generated held-out split; deployment data should be evaluated independently.

使用随机种子 `25012`、12 帧窗口、24 个隐藏单元和 300 轮训练的 12-episode 样例运行，在本机 CPU 测得 `MAE 0.0272`、`RMSE 0.0440`、`F1 0.774`、`准确率 0.969`、单窗口推理中位耗时 `0.0103 ms`。该结果仅对应本次生成的隔离测试划分，部署数据应独立评价。
