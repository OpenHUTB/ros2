# holoocean_depth_avoid

基于**深度图**（而非 RGB 灰度）的水下机器人前视避障与海底定高巡航模块。
运行于 **HoloOcean2 / ROS 2 Humble** 仿真环境。

## 功能

- 订阅深度相机图像（`sensor_msgs/Image`，`32FC1`，单位米）；
- 发布 `/auv/cmd_vel`（`geometry_msgs/Twist`）控制 6-DOF AUV/ROV；
- **前视避障**：取前向 ROI 最近深度，分三档决策
  - 前方 > warn_dist：巡航直行；
  - warn_dist ~ danger_dist：减速，向开阔侧转向绕开；
  - < danger_dist：触发脱困链（直退 → 原地转向 → 向前脱离），和防卡死漫游同思路但作用于水下；
- **海底定高**：取底部 ROI 中位深度，自动上浮/下潜，保持离底 `target_altitude` 米。

## 环境

- Ubuntu 20.04 / 22.04，ROS 2 Humble
- HoloOcean2（见仓库 [WSL_HoloOcean2_ROS2](../docs/water/WSL_HoloOcean2_ROS2.md) / Docker 教程）
- Python 3.10，依赖 `rclpy`、`sensor_msgs`、`geometry_msgs`、`cv_bridge`、`numpy`

## 编译与运行

```bash
# 1) 编译（把本包放到 ros2_ws/src 下）
cd ~/ros2_ws && colcon build --packages-select holoocean_depth_avoid
source install/setup.bash

# 2) 启动 HoloOcean2 仿真（带 DepthCamera 传感器的水下世界）
#    具体启动命令参见 HoloOcean2 文档 / 本仓库 water 类目教程

# 3) 确认深度话题名，并按需改 launch 里的 depth_topic
ros2 topic list | grep -i depth
ros2 topic hz /holoocean/auv0/DepthCamera

# 4) 启动本节点
ros2 launch holoocean_depth_avoid auv_depth_avoid.launch.py
```

## 可调参数（见 launch / `declare_parameter`）

| 参数 | 默认 | 含义 |
|---|---|---|
| `depth_topic` | `/holoocean/auv0/DepthCamera` | 深度相机话题（32FC1） |
| `cmd_topic` | `/auv/cmd_vel` | 速度发布话题 |
| `danger_dist` | 1.0 m | 触发脱困的前方最近距离 |
| `warn_dist` | 2.0 m | 减速绕开的前方最近距离 |
| `cruise_speed` | 0.8 | 巡航前进速度 |
| `target_altitude` | 1.5 m | 期望离底高度 |
| `heave_gain` | 0.4 | 定高垂向速度增益 |

## 运行效果

见文档页 [docs/water/holoocean_depth_avoid.md](../../docs/water/holoocean_depth_avoid.md)。
