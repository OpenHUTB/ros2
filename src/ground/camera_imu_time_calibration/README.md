# 相机-IMU 时间同步标定（camera_imu_time_calibration）

在**已知真值**的合成场景中估计相机与 IMU 之间的常量时间偏移，并量化"这点毫秒级偏移到底会造成多大的姿态误差"。

对应教程：[docs/ground/camera_imu_time_calibration.md](../../docs/ground/camera_imu_time_calibration.md)

## 这个模块补上了什么

`src/ground/carla_sensor_fusion` 里的"时间同步"是按**最近时间戳**对齐多传感器消息，前提是**时间戳本身可信**。本模块处理的是这个前提被打破的情况：相机时间戳与 IMU 时间戳之间存在未知的常量偏移（曝光延迟、传输延迟、时钟未对齐），需要**从数据本身估出来**。

`src/ground/lidar_camera_calibration` 解决的是**空间**对齐（外参），本模块解决**时间**对齐（时参），两者合起来才是完整的多传感器标定。

## 环境依赖

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 LTS（离线脚本在 Windows 上也可运行） |
| ROS | ROS 2 Humble（仅节点部分需要） |
| Python | 3.10.12 |
| NumPy | 1.21.5（ROS 2 Humble 自带版本） |
| Matplotlib | 3.5.1（仅绘图） |

```bash
python3 -m pip install --user -r src/ground/camera_imu_time_calibration/requirements.txt
```

离线脚本只依赖 NumPy；ROS 2 节点额外依赖 `rclpy`、`sensor_msgs`、`geometry_msgs`、`std_msgs`。

## 运行步骤

### 1. 离线仿真（不需要 ROS）

```bash
cd src/ground/camera_imu_time_calibration

# 标准演示：估计偏移 + 补偿前后姿态误差，输出 2 张图
python3 -m camera_imu_time_calibration.main

# 偏移量扫描 -200..+200 ms（21 档 × 20 次蒙特卡洛，约 36 s）
python3 -m camera_imu_time_calibration.sweep_offset

# 帧率与时间戳约定扫描（10..120 Hz，约 33 s）
python3 -m camera_imu_time_calibration.sweep_rate

# 鲁棒性实验：噪声 / 零偏 / 尺度 / 激励（约 26 s）
python3 -m camera_imu_time_calibration.sweep_noise

# 补偿前后姿态误差对比（约 28 s）
python3 -m camera_imu_time_calibration.compare_attitude
```

图片默认写入 `docs/img/ground/camera_imu_time_calibration/`，可用 `--out-dir` 改。

### 2. 单元测试

```bash
cd src/ground/camera_imu_time_calibration
python3 -m unittest discover -s test -v      # 34 项，约 1.5 s
```

`test_core.py` 覆盖算法（22 项），`test_params.py` 用假节点覆盖参数读取（8 项），两者都不需要 ROS。

### 3. 构建 ROS 2 包

```bash
cd ~/ros2_ws
colcon build --packages-select camera_imu_time_calibration --symlink-install
source install/setup.bash
```

### 4. 一键启动（无需硬件）

```bash
ros2 launch camera_imu_time_calibration main.launch.py
# 换一个偏移量再试（数值参数写整数也可以）
ros2 launch camera_imu_time_calibration main.launch.py offset_ms:=120 cam_rate:=15
```

launch 会启动两个节点：`camera_imu_synthetic_source`（合成数据源，把 50 ms 滞后"做进"相机角速度）和 `camera_imu_time_estimator`（估计并发布结果）。

```bash
# 查看估计结果
ros2 topic echo /camera_imu_time_estimator/time_offset
```

输出里的 `recent_std_ms` 是最近 `stability_window`（默认 10）次重估的标准差，`stable` 表示它是否在 `stability_warn_ms`（默认 2 ms）以内——用来判断"常量偏移"这个假设是否还成立（详见教程 7.4 节）。

### 5. 接真实数据

把 `imu_topic` 指到真实 IMU，`cam_topic` 指到视觉里程计输出的角速度：

```bash
ros2 run camera_imu_time_calibration estimator_node --ros-args \
  -p imu_topic:=/camera/imu \
  -p cam_topic:=/vio/angular_velocity \
  -p window_sec:=10
```

## 文件说明

| 文件 | 作用 |
| --- | --- |
| `camera_imu_time_calibration/core.py` | 轨迹真值、传感器模型（曝光时长与帧周期分开）、时间偏移估计器、姿态误差 |
| `camera_imu_time_calibration/ros_params.py` | 参数读取辅助：数值参数用动态类型声明，`-p x:=10` 这类整数覆盖不会崩 |
| `camera_imu_time_calibration/main.py` | 标准演示入口 |
| `camera_imu_time_calibration/sweep_offset.py` | 偏移量扫描 |
| `camera_imu_time_calibration/sweep_rate.py` | 帧率、时间戳约定与曝光时长扫描 |
| `camera_imu_time_calibration/sweep_noise.py` | 鲁棒性实验 |
| `camera_imu_time_calibration/compare_attitude.py` | 补偿前后姿态误差 |
| `camera_imu_time_calibration/estimator_node.py` | 在线估计节点（含重估稳定性指标） |
| `camera_imu_time_calibration/synthetic_source_node.py` | 演示用合成数据源 |
| `launch/main.launch.py` | 一键启动 |
| `requirements.txt` | NumPy / Matplotlib 版本约束 |
| `test/test_core.py` | 26 项算法单元测试（含尺度比去零偏、滑窗裁剪口径） |
| `test/test_params.py` | 8 项参数读取单元测试 |

## 主要结果（实跑数据，详见教程）

运行环境：Windows 11 + Python 3.13 + NumPy 2.5.4（下表数值由算法决定，与平台无关）。
ROS 2 节点部分已由评审在 Ubuntu 22.04 + Humble 实跑，状态见教程第 8 节。

* 30 Hz 相机、50 ms 真值偏移：估计 50.30 ms，误差 **0.30 ms**，互相关峰值 0.9997；
* 偏移量 ±200 ms 全范围：斜率 1.0005，最大误差 **0.16 ms**，重复性 std 0.35 ms；
* 同一 50 ms 偏移下姿态误差：未补偿 RMS **3.36°** → 补偿后 **0.72°**（降低 79%）；
* IMU 零偏从 0 加到 0.5 rad/s，估计误差**完全不变**（去均值使其免疫）；
* 相机角速度尺度误差 1.3 倍，偏移估计**不受影响**，且尺度比被独立估出（1.298）；尺度比用去均值回归，对陀螺常值零偏免疫；
* 帧首时间戳的偏差等于 **$T_{exp}/2$**（曝光时长的一半，不是固定半帧）：30 Hz 下曝光 3.3 ms 时偏 -1.56 ms、曝光 33.3 ms 时偏 -16.70 ms；时间戳取曝光中点时误差恒为 0.07 ms，与曝光长短无关。
