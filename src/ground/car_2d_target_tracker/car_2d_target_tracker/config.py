#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""参数集中定义（与原实现常量一一对应，便于对照）。

原实现的常量散落在 main.py 顶部，且部分与渲染尺寸混在一起（WIDTH/HEIGHT 既是
场地大小又是窗口大小）。这里把"物理/控制"与"渲染"分开：headless 评测只用到
``TrackerConfig``，像素尺寸只影响绘图。
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class TrackerConfig:
    """场地、车辆、控制与判据参数。"""

    # --- 场地 ---
    width: float = 900.0
    height: float = 600.0

    # --- 车辆 ---
    max_speed: float = 180.0       # px/s
    max_steer: float = 3.5         # rad/s
    kp_turn: float = 2.5           # 转向比例增益
    car_radius: float = 10.0
    arrive_radius: float = 18.0

    # --- 控制细节 ---
    slow_radius: float = 120.0     # 距离小于它开始减速（原实现里的 120）
    # 改进版的减速半径：原实现用 120 px，靠近时速度被压到与移动目标同量级
    # （实测追击 60 px/s 的目标时末距停在 40 px 永远追不上），改进版只在快到
    # 达时才减速，保证仍有追赶余量。
    chase_taper: float = 40.0
    dt: float = 1.0 / 60.0         # 仿真步长（= 原实现 60 FPS）

    # --- 障碍物 (x, y, r) ---
    obstacles: tuple = ((300.0, 200.0, 35.0),
                        (600.0, 380.0, 45.0),
                        (450.0, 450.0, 30.0))

    # --- 改进版控制器的绕障行为 ---
    # 0.85 是实测选出来的：0.35 时绕障爬得太慢（blocked 场景 16.9 s），
    # 0.7 起进入平台（10.5 s / 10.3 s / 10.2 s），取 0.85 兼顾速度与贴墙余量。
    escape_speed_ratio: float = 0.85   # 被挡时沿切向前进的速度比例（0 = 原实现"原地打转"）
    escape_steer_ratio: float = 1.0    # 被挡时转向强度比例
    stuck_speed_eps: float = 5.0       # 低于此速度视为"几乎没动" (px/s)
    stuck_time: float = 1.0            # 连续这么久几乎没动 → 判定卡死 (s)
    stuck_flip_time: float = 0.8       # 卡死后反向绕行持续多久 (s)

    # --- 目标运动 ---
    target_speed: float = 60.0
    target_margin: float = 25.0

    # --- 场景 ---
    sim_time: float = 30.0
    seed: int = 42

    extra: dict = field(default_factory=dict)


@dataclass
class Scenario:
    """一个评测场景：初始位姿 + 目标运动方式 + 可用障碍。"""

    name: str
    car: tuple                    # (x, y, theta)
    target: tuple                 # (x, y)
    target_motion: str = "wander"  # wander | static
    obstacles: tuple = ()
    sim_time: float = 30.0
    note: str = ""
