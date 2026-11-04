#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2D 小车目标追踪（纯运动学，不依赖渲染库）。

模块划分：

    config.py      参数与场景定义
    geometry.py    角度归一化、圆/线段距离、扫掠碰撞（纯函数，可单测）
    car.py         单轮车运动学模型（整段扫掠、贴住障碍不穿模）
    target.py      目标运动（固定 seed 可复现）
    controller.py  两种控制器：baseline（复原原实现）/ escape（绕障改进）
    metrics.py     到达时间、路径长度、最小间隙、卡死占比（纯函数）
    runner.py      三场景对照实验 + CSV 导出
    plotting.py    matplotlib 可选绘图
    gui.py         pygame 可选可视化
    cli.py         命令行入口

设计原则：**渲染是可选依赖**。默认路径（headless）只用标准库 + numpy，
因此在没有 pygame、没有图形界面的评测机上也能跑出全部指标。
"""

__version__ = "1.0.0"

from car_2d_target_tracker.config import Scenario, TrackerConfig  # noqa: F401
from car_2d_target_tracker.car import Car                          # noqa: F401
from car_2d_target_tracker.controller import Tracker              # noqa: F401
from car_2d_target_tracker.target import Target                    # noqa: F401
