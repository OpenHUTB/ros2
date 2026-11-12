# mag_hard_soft_iron_calibration

磁力计硬磁/软磁标定与倾角补偿航向：把旋转测量拟合成椭球 → 求偏置与软磁矩阵 →
用对称平方根消除旋转自由度 → 与陀螺做一维卡尔曼融合。

## 环境

- 离线：Windows 11、Python 3.12.14、NumPy 2.3.5、Matplotlib 3.11.2（仅绘图）
- ROS 2：Ubuntu 22.04 + ROS 2 Humble

## 构建与运行（ROS 2）

```bash
source /opt/ros/humble/setup.bash
cd ~/hutb_ws
colcon build --symlink-install --packages-select mag_hard_soft_iron_calibration
source install/setup.bash
ros2 launch mag_hard_soft_iron_calibration main.launch.py samples:=600
```

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/mag/data` | `sensor_msgs/MagneticField` | 输出（仿真） | 带硬磁/软磁畸变的测量 |
| `/mag/attitude` | `sensor_msgs/Imu` | 输出（仿真） | 用于倾角补偿的重力向量 |
| `/mag/status` | `std_msgs/String` | 输出（标定） | 偏置、软磁矩阵、残差与航向（JSON） |

## 离线实验

```bash
cd src/ground/mag_hard_soft_iron_calibration
python3 main.py --out results --figure-dir ../../../docs/img/ground/mag_hard_soft_iron_calibration
python3 -m unittest discover -s tests -v
```

## 主要结果

| 指标 | 数值 |
|---|---|
| 偏置误差 | 0.90 µT（硬磁真值 26.1 µT） |
| 软磁矩阵相对误差 | 1.1 % |
| 拟合残差 RMS | 0.46 µT（噪声 0.5 µT） |

| 航向来源 | RMSE | 最大误差 |
|---|---|---|
| 原始磁力计 | 42.65° | 91.51° |
| 仅硬磁补偿 | 10.53° | 22.41° |
| 椭球标定 | 1.26° | 4.51° |
| 陀螺 + 磁力计融合 | **0.33°** | **1.17°** |

详见 `docs/ground/mag_hard_soft_iron_calibration.md`。
