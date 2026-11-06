# carla_slam_nav

地面载具（CARLA）SLAM 建图 + 自主导航同步仿真（任务三）。

- 激光雷达 → RTAB-Map 6 位姿 SLAM（输出 `/map` 与 `odom→base` TF）
- Nav2（NavFn 全局规划 + DWB 局部控制 + 代价地图）在同一仿真时钟下导航
- 建图与导航同步闭环，支持多场景（Town01/02/04）

启动：

```bash
# 1. 先起 CARLA + ros-bridge（带激光雷达的 ego 车辆）
# 2. 建图
ros2 launch carla_slam_nav carla_slam.launch.py
# 3. 另开终端起导航
ros2 launch carla_slam_nav carla_nav2.launch.py
# 4. RViz 中给 /initialpose 与 /move_base_simple/goal
```

教程：`docs/ground/slam_nav.md`。
