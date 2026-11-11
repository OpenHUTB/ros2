# imu_allan_noise_identification

从一段静止的陀螺仪记录里用 Allan 方差辨识角度随机游走（ARW）与零偏不稳定性（BI），
再把两者分别换算成两状态倾角卡尔曼的角度过程噪声与偏置随机游走；同时给出消融实验与
"凭经验调参"的对照组，用 NEES 说明为什么不能随手填 Q。

## 环境

- 离线部分：Windows 11、Python 3.12.14、NumPy 2.3.5、Matplotlib 3.11.2（仅绘图）
- ROS 2 部分：Ubuntu 22.04 + ROS 2 Humble（Python 3.10）

## 构建与运行（ROS 2）

```bash
source /opt/ros/humble/setup.bash
cd ~/hutb_ws
colcon build --symlink-install --packages-select imu_allan_noise_identification
source install/setup.bash
ros2 launch imu_allan_noise_identification main.launch.py duration:=120
```

`imu_sim` 以 100 Hz 回放，`allan_identifier` 默认收满 `max_samples: 12000`（=120 s）后自动辨识。
`duration:=120` 只够**演示流程**：120 s 记录的最长簇时长只有 13 s，ARW 可估、**BI 不可估**，
此时节点会打印 `bias instability NOT identifiable` 警告并在 `/imu/allan` 里给出 `bias_reliable: false`，
`bias_instability_deg_h` 可能是 0.0（拟合把负的 BI² 截断），不能当结果用。
要真正辨识 BI，请用 `duration:=3600` 并同步 `max_samples: 360000`（首次运行需回放 1 h）；
若把 `max_samples` 调大，记得同步加长 `duration`。中途 Ctrl-C 也会对已收样本做一次辨识（样本过少时只打 warning）。

话题（单位遵循 `sensor_msgs/Imu`）：

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | 输出（仿真） | `angular_velocity.x` 为 rad/s 的横滚角速度；`linear_acceleration` 为 m/s² 的重力分量；`orientation` 为绕 x 的四元数 |
| `/imu/allan` | `std_msgs/String` | 输出（辨识） | 采样数、采样率、ARW、BI 与曲线点数（JSON） |

参数（`config/demo.yaml`）：`rate_hz`、`duration_s`、`seed`、`max_samples`、`cluster_points`、`output_csv`。
两个节点共用同一个 `rate_hz`，辨识节点的 `dt` 由它推出，不再硬编码。

## 离线实验（不需要 ROS）

```bash
cd src/ground/imu_allan_noise_identification
python3 -m pip install -r requirements.txt
python3 main.py --out results --figure-dir ../../../docs/img/ground/imu_allan_noise_identification
python3 -m unittest discover -s tests -v
```

## 结果（1 h 静止记录，100 Hz）

| 参数 | 真值 | 单次辨识 | 10 种子中位数 [95 % 区间] |
|---|---|---|---|
| ARW [deg/√h] | 0.300 | 0.3015（+0.5 %） | **+0.46 %** [+0.23, +0.92] |
| 零偏不稳定性 [deg/h] | 3.00 | 3.485（+16.2 %） | **+3.77 %** [−16.70, +44.42] |

| 滤波器 | RMSE [°] | NEES 均值 |
|---|---|---|
| 按 Allan 整定（ARW → 角度噪声，BI → 偏置游走） | 0.0170 | **1.61** |
| ARW 与 BI 都偏小 4 倍 | 0.0179 | 10.07 |
| 消融：ARW 除以 4 | 0.0179 | 7.82 |
| 消融：BI 乘以 60 | 0.0195 | 1.55 |

详细推导、消融与限制见 `docs/ground/imu_allan_noise_identification.md`。
