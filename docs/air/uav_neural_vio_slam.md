# 神经视觉惯性里程计与三后端 SLAM 对比

!!! note "占位页"
    完整内容随本模块 PR 提交。

对应任务（2）传感器感知、任务（3）建图导航，命中对比实验①（同模型、不同 SLAM 算法）。
GPS 拒止下用相机 + IMU 做神经 VIO，对比三种 SLAM 后端。

## 与已有模块的区别

| 已有 | 区别 |
|---|---|
| `octree_uav_3d_pathfinding` | 直接用真值位姿建图，无状态估计、无 SLAM 后端 |
| `carla_slam_gmapping` | 是车；gmapping 为黑箱，做不了"同前端换后端"的受控对比 |

课程内尚无"状态估计与建图分开评测"的工作。

## 计划

* 前端 CNN + GRU：IMU 序列 + 下采样深度图 → 相对位姿增量
* 后端（同一前端、同一数据集）：EKF-SLAM / FastSLAM / 神经观测模型 SLAM
* 位姿 + 激光雷达 → 占用栅格，rviz 实时可视化
* 对比：ATE / RPE、地图 IoU、GPS 中断 5 / 15 / 30 s 的退化曲线

依赖：[uav_mujoco_sim](uav_mujoco_sim.md)
