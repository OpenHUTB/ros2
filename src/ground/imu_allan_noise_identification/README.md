# imu_allan_noise_identification

从一段静止的陀螺仪记录里用 Allan 方差辨识角度随机游走（ARW）与零偏不稳定性（BI），
再把辨识结果直接换成两状态倾角卡尔曼的过程噪声；同时给出一个"凭经验调参"的对照组，
用 NEES 说明为什么不能随手填 Q/R。

## 环境

- 离线部分：Windows 11、Python 3.12.14、NumPy 2.3.5、Matplotlib 3.11.2（仅绘图）
- ROS 2 部分：Ubuntu 22.04 + ROS 2 Humble

## 构建与运行（ROS 2）

```bash
source /opt/ros/humble/setup.bash
cd ~/hutb_ws
colcon build --symlink-install --packages-select imu_allan_noise_identification
source install/setup.bash
ros2 launch imu_allan_noise_identification main.launch.py duration:=120
```

话题：

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | 输出（仿真） | 100 Hz 陀螺 z 轴、横滚四元数与加速度倾角 |
| `/imu/allan` | `std_msgs/String` | 输出（辨识） | 采样数、ARW、BI 与曲线点数（JSON） |

参数（`config/demo.yaml`）：`rate_hz`、`duration_s`、`seed`、`max_samples`、`cluster_points`、`output_csv`。

## 离线实验（不需要 ROS）

```bash
cd src/ground/imu_allan_noise_identification
python3 -m pip install -r requirements.txt
python3 main.py --out results --figure-dir ../../../docs/img/ground/imu_allan_noise_identification
python3 -m unittest discover -s tests -v
```

## 结果（1 h 静止记录，100 Hz）

| 参数 | 真值 | 辨识值 | 误差 |
|---|---|---|---|
| ARW [deg/√h] | 0.300 | 0.3015 | +0.5 % |
| 零偏不稳定性 [deg/h] | 3.00 | 3.485 | +16.2 % |

| 滤波器 | RMSE [°] | 最大误差 [°] | NEES 均值 |
|---|---|---|---|
| 按 Allan 整定 | 0.0170 | 0.434 | 1.62 |
| 凭经验调（Q/R 偏小 4 倍） | 0.0179 | 0.434 | 8.70 |

详细方法、记录长度对比与限制见 `docs/ground/imu_allan_noise_identification.md`。
