# 第三方代码声明（THIRD PARTY NOTICES）

本包 `humanoid_push_recovery/balance/` 子包中的部分文件改写自下列开源项目：

- 项目：<https://github.com/PrageethM1702/Bipedal-Walking-and-Push-Recovery>
- 许可证：Apache License 2.0（原文随附于本目录 `LICENSE_apache_2_0.txt`）
- 版权：Copyright (c) the original authors

## 改写的文件与本站改动

| 本站文件 | 上游文件 | 主要改动 |
|---|---|---|
| `balance/capture_point.py` | `balance/capture_point.py` | 入参放宽为任意维位置/速度向量；增加 `capture_point_offset` |
| `balance/support_polygon.py` | `balance/support_polygon.py` | 顶点来源改为本模型脚掌 geom 的 8 个角点；几何方法同时以模块级函数暴露 |
| `balance/ankle_strategy.py` | `balance/ankle_strategy.py` | 输出由「关节目标角偏置」改为**关节力矩 (N·m)**；倾角输入改用躯干轴倾角与低通速率；增加质心位置/速度反馈项以消除稳态偏差 |
| `balance/hip_strategy.py` | `balance/hip_strategy.py` | 输出改为关节力矩；保留已整定的质心位置反馈项；松弛滤波默认关闭 |
| `balance/controller.py` | （参考其 `control/main_controller.py` 的分级结构） | 按本站 MuJoCo 力矩控制模型重写 |

以上改动均为适配「MuJoCo 力矩控制模型 + 本仓库 ROS2 包结构」所必需，
依据 Apache License 2.0 的约定在此声明修改。

## 本站新增（非来自上游）

`balance/__init__.py`、`humanoid_push_recovery/sim_core.py`、`evaluate.py`、
`mujoco_sim_node.py`、`push_teleop_node.py`、`main.py`、`test/`、`models/`、
`launch/`、`config/` 均为本站编写。
