# MuJoCo 差速小车轮滑感知与轨迹补偿

MuJoCo Differential-Drive Slip Perception and Trajectory Compensation

本 ROS 2 功能包提供一个四轮差速小车的 MuJoCo 仿真基础：当前入口接收 `cmd_vel`，通过差速运动学计算四轮目标转速，并发布仿真时钟、真实里程计和坐标变换；同时发布带可复现噪声的编码器和惯性测量单元数据。多摩擦采集入口按完整 episode 轮换场景、复位车辆、执行确定性激励指令，并把同步观测写入按 episode 隔离的数据文件。

This ROS 2 package provides a MuJoCo simulation foundation for a four-wheel differential-drive robot. The entry point consumes `cmd_vel`, converts chassis commands to four wheel-speed targets, and publishes simulation time, ground-truth odometry, and transforms. It also publishes encoder and inertial measurement unit data with reproducible noise. The multi-friction collection entry point rotates scenarios by complete episode, resets the vehicle, executes a deterministic excitation profile, and writes synchronized observations to episode-isolated files.

## 环境 / Environment

| 组件 / Component | 版本 / Version |
| --- | --- |
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| Python | 3.10 |
| MuJoCo | 3.12.0 |
| PyTorch | CPU build, required by later training work / 后续训练阶段使用 CPU 版本 |

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
│   ├── resources.py
│   ├── scenarios.py
│   ├── sensors.py
│   └── simulator.py
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

￼

102

​

￼

103

## 多摩擦数据采集 / Multi-friction Data Collection

￼

104

​

￼

105

默认场景使用 `0.25`、`0.55`、`0.90` 三个滑动摩擦系数，每个 episode 持续 30 秒。固定随机种子 `25012` 控制传感器噪声和数据集划分。

￼

106

​

￼

107

The default scenarios use sliding-friction coefficients `0.25`, `0.55`, and `0.90`, with each episode lasting 30 seconds. The fixed random seed `25012` controls sensor noise and dataset assignment.

￼

108

​

￼

109

```bash

￼

110

ros2 launch mujoco_diff_drive_slip_compensation collect.launch.py \

￼

111

output_directory:=data/raw

￼

112

```

￼

113

​

￼

114

每个 episode 生成同名 `.csv` 和 `.json`。CSV 保存指令、四轮编码器、IMU、摩擦系数和真实位姿；JSON 记录数据格式版本、划分、行数、随机种子、Python、MuJoCo 和功能包版本。`data/` 默认被 Git 忽略。

￼

115

​

￼

116

Each episode produces matching `.csv` and `.json` files. The CSV stores commands, four-wheel encoders, IMU measurements, friction, and ground-truth pose. The JSON records schema version, split, row count, random seed, and Python, MuJoCo, and package versions. `data/` is ignored by Git by default.

￼

117

​

￼

118

训练、验证、测试划分由 `seed:episode_id` 的 SHA-256 哈希确定，默认比例为 70%/15%/15%。一个 episode 的全部相邻帧始终属于同一划分。

￼

119

​

￼

120

Train, validation, and test assignment uses the SHA-256 hash of `seed:episode_id`, with default ratios of 70%/15%/15%. Every adjacent frame from one episode always remains in the same split.

￼

121
