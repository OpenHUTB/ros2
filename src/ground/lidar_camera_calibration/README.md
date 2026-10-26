# 相机-激光雷达外参标定（ROS 2 Humble）

在**已知真值的合成场景**中完成 LiDAR → 相机外参标定：多姿态棋盘格点云 →
加传感器噪声 → PnP 求解 → 与真值对比，给出旋转/平移误差与重投影残差。
同时提供噪声敏感性、标定板摆位次数两项定量实验，以及 ROS 2 节点入口。

> 本模块的重点是**误差可量化**：合成场景有真值，所以能直接回答"标得有多准"。
> 真实传感器数据采集与棋盘格检测列在文档"未验证项"。

## 依赖

- Ubuntu 22.04 + ROS 2 Humble（节点部分）
- Python 3.10
- `numpy`、`opencv-python`、`matplotlib`

```bash
python3 -m pip install --user -r requirements.txt
```

## 构建与运行

```bash
cd ~/ros2_ws
colcon build --packages-select lidar_camera_calibration --symlink-install
source install/setup.bash

# 单次标定（3 个摆位，1 px + 1 cm 噪声）
ros2 run lidar_camera_calibration run_calibration --views 3

# 摆位次数扫描
ros2 run lidar_camera_calibration sweep_views --trials 200

# 噪声敏感性扫描
ros2 run lidar_camera_calibration sweep_noise --trials 200

# 生成投影叠加图与误差曲线（默认输出到 /tmp/lidar_camera_calibration）
ros2 run lidar_camera_calibration project_cloud --out-dir /tmp/calib_out

# ROS 2 节点（订阅 /camera_info，发布标定结果）
ros2 launch lidar_camera_calibration main.launch.py
```

不构建也可以直接跑（在模块目录下）：

```bash
python3 -m lidar_camera_calibration.run_calibration --views 3
```

## 文件

| 文件 | 作用 |
| --- | --- |
| `lidar_camera_calibration/calibration_core.py` | 场景生成、LiDAR 噪声模型、PnP 求解、误差指标、投影叠加 |
| `lidar_camera_calibration/run_calibration.py` | 单次标定，输出估计值与真值对照 |
| `lidar_camera_calibration/sweep_noise.py` | 像素 × 测距噪声扫描 |
| `lidar_camera_calibration/sweep_views.py` | 标定板摆位次数扫描 |
| `lidar_camera_calibration/project_cloud.py` | 生成投影叠加图与两张误差曲线图 |
| `lidar_camera_calibration/calibration_node.py` | ROS 2 节点：读 `/camera_info`，发布 `/lidar_camera_calibration/extrinsics` |
| `lidar_camera_calibration/main.py` | 统一入口（标准演示） |
| `launch/main.launch.py` | 一键启动节点 |

## 实测结果（Ubuntu 22.04 + ROS 2 Humble + OpenCV 4.5.4）

单次标定（3 摆位、105 点、1 px + 1 cm + 0.1° 噪声）：

| 指标 | 结果 |
| --- | --- |
| 旋转误差 | 0.218° |
| 平移误差 | 4.6 mm |
| 重投影 RMSE | 2.35 px |

摆位次数扫描（100 次蒙特卡洛中位数）：1 → 2.41°／4.2 cm，3 → 0.15°／3.2 mm，
6 → 0.10°／2.1 mm。**单板共面时误差显著变大**，这是本模块最想说明的现象。

噪声敏感性（3 摆位）：0 px + 0 cm → 0.11°／2.1 mm；2 px + 2 cm → 0.23°／4.8 mm。

详细原理、参数与排错见
[docs/ground/lidar_camera_calibration.md](../../../docs/ground/lidar_camera_calibration.md)。
