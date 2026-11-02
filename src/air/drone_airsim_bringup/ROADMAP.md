# CarlaAir-Drone 总进度

**项目：** 空地一体无人机自主飞行仿真实验（CarlaAir-Drone）
**后端：** CarlaAir（CARLA 0.9.16 + AirSim 1.8.1）+ ROS Noetic
**目标：** 在同一个城市仿真里完成无人机的物理控制、感知、建图导航和端到端飞行，并满足课程对 launch、神经网络、文档和 PR 审核的要求。

## 四阶段 / 11 个 PR

| 阶段 | PR | 内容 | 状态 |
|---|---|---|---|
| 阶段 1 | PR1 | `drone_airsim_bringup`：位姿 + 相机 + takeoff/land | 已按 review 修改，待复审 |
| 阶段 1 | PR2 | 深度/IMU 传感器 + 10 Hz 采集 | 未开始 |
| 阶段 1 | PR3 | 键盘遥控 + 起飞/降落 | 未开始 |
| 阶段 2 | PR4 | 数据采集 + YOLOv8n 训练 | 未开始 |
| 阶段 2 | PR5 | ROS 感知节点（`/drone/detections`） | 未开始 |
| 阶段 2 | PR6 | 轨迹跟踪控制器（PID/MPC） | 未开始 |
| 阶段 3 | PR7 | SLAM（RTAB-Map / ORB-SLAM3） | 未开始 |
| 阶段 3 | PR8 | 占据栅格 + 航点导航 | 未开始 |
| 阶段 4 | PR9 | 端到端数据 + CNN 训练 | 未开始 |
| 阶段 4 | PR10 | 端到端 ROS 节点 + 全链路集成 | 未开始 |
| 阶段 4 | PR11 | mkdocs 文档 + Word + GIF | 未开始 |

## 本 PR（PR1）在整体中的位置

PR1 只做“能把无人机接进 ROS”：位姿、相机、速度控制、起降服务。它不包含感知、SLAM 和端到端；这些在 PR2–PR11 里逐步加。

## 验收口径

- 每个模块都能用 `main.py` / `launch` 直接启动；
- 在 Ubuntu 20.04 + ROS Noetic 上可复现；
- 每个 PR 附运行输出和已知限制；
- 至少 10 次提交/审核；
- 文档包含公式、流程、源码解析和仿真截图。

## 当前可演示内容

- CarlaAir：Town10HD、30 车 + 50 行人、无人机起飞/移动/降落；
- ROS：`/drone/pose`、`/drone/camera/image_raw`、`/drone/cmd_vel`、takeoff/land；
- 标准 CARLA + ROS bridge：`/carla/status`、`/carla/world_info`（另一条线，用于车辆/ros-bridge 验收）。
