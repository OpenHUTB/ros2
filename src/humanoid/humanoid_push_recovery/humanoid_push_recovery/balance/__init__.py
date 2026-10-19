# -*- coding: utf-8 -*-
"""平衡判据与抗扰恢复控制子包（无 ROS 依赖）。

本子包中的**捕获点判据、支撑多边形几何、踝策略、髋策略**改写自开源项目：

    PrageethM1702/Bipedal-Walking-and-Push-Recovery
    https://github.com/PrageethM1702/Bipedal-Walking-and-Push-Recovery
    Copyright (c) the original authors, Apache License 2.0

改写点（均为适配本站 MuJoCo 力矩控制模型所必需，逐文件在文件头注明）：
  1. 原项目基于 PyBullet 的位置控制机器人，策略输出「关节目标角偏置」；
     本站模型是 MuJoCo 力矩控制机器人，因此策略输出改为**关节力矩 (N·m)**；
  2. 原项目的机器人接口（``robot.get_com_position()`` 等）替换为本模块
     ``BalanceSim`` 的状态读取接口；
  3. 支撑多边形的顶点来源由「脚部矩形近似」改为**本模型脚掌 geom 的 8 个角点**；
  4. 髋策略在原「质心速度反馈」基础上保留本站已整定的质心位置项。

详细声明见模块根目录 ``THIRD_PARTY_NOTICES.md``。

对外接口
--------
几何 / 判据：``convex_hull``、``point_in_polygon``、``distance_to_boundary``、
             ``SupportPolygon``、``capture_point``、``omega``、``stability_margin``
控制器：      ``AnkleStrategy``、``HipStrategy``、``BalanceController``
"""

from .capture_point import capture_point, omega
from .support_polygon import (
    SupportPolygon,
    convex_hull,
    distance_to_boundary,
    point_in_polygon,
)
from .ankle_strategy import AnkleStrategy
from .hip_strategy import HipStrategy
from .controller import BalanceController, stability_margin

__all__ = [
    "capture_point", "omega",
    "SupportPolygon", "convex_hull", "point_in_polygon", "distance_to_boundary",
    "AnkleStrategy", "HipStrategy",
    "BalanceController", "stability_margin",
]
