# uav_depth_track_nn

作业任务 2：深度相机感知 + 给定圆轨迹跟踪（CNN 行为克隆），ROS 2 Humble。

## 环境

* Ubuntu 22.04 + ROS 2 Humble + Python 3.10
* numpy / opencv-python / torch（CPU）
* OpenHUTB 2.10.0 或 AirSim Blocks，RPC 端口 8080

## 编译

```bash
cd ~/ros2
colcon build --packages-select uav_depth_track_nn
source install/setup.bash
```

## 采集

```bash
ros2 launch uav_depth_track_nn collect.launch.py airsim_port:=8080 vehicle_name:=drone1
```

自动起飞、沿半径 3 m 高度 2.5 m 的圆飞两圈、自动降落，数据写入
`~/.ros/uav_depth_track_nn/data/`。

## 训练

```bash
python3 -m uav_depth_track_nn.train --epochs 60 --batch-size 16
```

权重与指标写入 `~/.ros/uav_depth_track_nn/model/`。

## 闭环跟踪

```bash
ros2 launch uav_depth_track_nn main.launch.py airsim_port:=8080 vehicle_name:=drone1
ros2 service call /uav/takeoff std_srvs/srv/Trigger
```

## 本机实测

| 指标 | 数值 |
| --- | --- |
| 采集样本 | 555 |
| 参数量 | 556,354 |
| 训练 | 67.3 s, 60 epoch |
| 测试 MSE / MAE | 0.0124 (m/s)^2 / 0.0716 m/s |
| 半径均值 / RMSE | 3.36 m / 0.488 m |
| 高度 RMSE | 0.305 m |

详细原理、公式与源码解析见 `docs/air/uav_depth_track_nn.md`。