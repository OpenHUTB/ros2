# 第三方代码声明（THIRD PARTY NOTICES）

本包 `humanoid_local_planner_bench/planners.py` 中的 DWA 与 APF 两种实现，
改写自下列开源项目：

- 项目：<https://github.com/AtsushiSakai/PythonRobotics>
- 许可证：MIT License（原文随附于本目录 `LICENSE_MIT.txt`）
- 版权：Copyright (c) 2016 Atsushi Sakai

## 改写的文件与本站改动

| 本站文件 | 上游文件 | 主要改动 |
|---|---|---|
| `planners.py` → `DWAPlanner` | `PathPlanning/DynamicWindowApproach/dynamic_window_approach.py` | 去掉 matplotlib 演示与全局路径依赖；障碍输入由「已知地图」改为**实时激光点**；代价函数改为**先剔除会撞的轨迹（硬约束）、再在可行解里择优**（上游为加权代价，直接照搬会因符号导致「越贴障越优」）；输出统一为 `(v, ω)` |
| `planners.py` → `APFPlanner` | `PathPlanning/PotentialFieldPlanning/potential_field_planning.py` | 同样改为激光输入与 `(v, ω)` 输出；**新增局部极小值检测**（2 s 无进展则叠加 1.5 s 切向逃逸），否则会在 0.90 m 窄门处卡死；斥力参数按本场景重新整定 |

以上改动均为适配「本仓库 ROS 2 包结构 + 人形足迹窄门场景」所必需，
依据 MIT License 的约定在此声明。

## 本站新增（非来自上游）

- `sim_core.py` —— 场景加载、运动学位姿写入、激光射线、间距与碰撞查询
- `metrics.py` —— 指标统计、CSV 导出、轨迹对比图
- `figures.py` —— 多机位离屏渲染与拼图
- `ros_node.py` —— ROS 2 节点（scan / odom / status / TF）
- `models/corridor_scene.xml` —— 走廊、窄门、障碍、人形机器人与运动学步态
- `test/test_standalone.py` —— 10 项自检
