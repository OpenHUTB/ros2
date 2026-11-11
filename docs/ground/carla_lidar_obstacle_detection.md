# 激光雷达 3D 目标检测

> 模块路径：`src/ground/carla_lidar_obstacle_detection` ｜ 载体：地面车 ｜ 地图：Town10HD_Opt

## 实验目的

把语义激光雷达的原始点云，转换成「目标级」输出（类别 + 质心 + 包围盒 + 距离），
并以仿真器真值对账，量化检测召回率。

## 方法

1. **数据**：`sensor.lidar.ray_cast_semantic`，128 线、50 m、120 万点/秒、20 Hz。
   每个点 6 个 32 位字：`x, y, z, cos(incidence), ObjIdx(u32), ObjTag(u32)`。
   **后两个是整数不是浮点**，必须按 `uint32` 解析再把前 3 个重解释成 `float32`，
   否则实例号会变成 1e-44 量级的「极小浮点数」，分组永远失败。
2. **目标布置**：在自车周围 8~35 m 的地图出生点上摆 6 辆车（出生点天然落在车道中心，互相可见性最好）。
3. **检测**：按 `ObjIdx` 分组 → 过滤点数 < 3 的组 → 计算质心、包围盒、到自车距离；
   类别先用 `world.get_actor(ObjIdx).type_id` 归一化，拿不到再用 `ObjTag` 兜底。
4. **多帧累积**：激光雷达水平角分辨率有限（一圈只有几十个采样），单帧经常扫不到车；
   累积最近 5 帧点云再检测，召回率显著提升（14 帧统计）。
5. **对账**：统计 60 m 内真实存在的车辆/行人数，算召回率。

## 结果

| 指标 | 数值 |
|---|---|
| 累积点云规模 | 约 4.1 万点/帧（5 帧累积） |
| 60 m 内真值 | 6 辆车 |
| 最佳帧检出 | 6 辆车（召回率 1.00） |
| 平均召回率（14 帧，非最佳帧） | **0.73**（阈值 0.60） |
| 平均每帧检出 | 5.6 个目标（其中车辆 4.4 辆） |
| 判定 | PASS |

BEV 检测图见 `src/ground/carla_lidar_obstacle_detection/docs/bev_detection.png`，
目标明细见同目录 `detections.csv`。

## 结论

- 用「实例号分组 + 多帧累积」可以在稀疏点云上稳定拿到多目标，且类别与真值完全一致。
- 单帧漏检是**角分辨率**导致的、不是算法问题：累积 5 帧后平均召回率约 0.73（最佳帧 1.00）。
- 验收改为「14 帧平均召回率 ≥ 0.60 且平均每帧车辆 ≥ 3.0」，避免只看最佳帧的樱桃挑选。

## 局限与改进

1. 依赖语义激光雷达的实例号；换成普通 `ray_cast` 需要实现地面分割 + 欧氏聚类。
2. 未做跨帧跟踪，目标数统计只在单帧意义上有效。
3. 只区分 vehicle / walker / other 三类，未做细粒度分类。

## 运行效果（截图 / 动图）

![激光雷达 3D 检测（BEV 视图 + 逐帧检出统计） 动图](docs/img/ground/carla_lidar_obstacle_detection_run.gif)

![激光雷达 3D 检测（BEV 视图 + 逐帧检出统计） 截图 kf1](docs/img/ground/carla_lidar_obstacle_detection_kf1.png)

![激光雷达 3D 检测（BEV 视图 + 逐帧检出统计） 截图 kf2](docs/img/ground/carla_lidar_obstacle_detection_kf2.png)

> 动图由实际运行的帧合成（GIF < 5 MB）；素材与生成脚本见 `tools/build_media.py`、`tools/build_lidar_media.py`、`tools/build_air_media.py`。
