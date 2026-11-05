#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""追踪控制器：两种模式，用来做对照。

``baseline``（复原原实现）
    纯"朝向目标 + 前进"：角误差乘比例增益得到角速度，速度按 ``cos(误差)`` 与距离
    衰减。撞到障碍时**把速度清零、原地按最大角速度打转**——原实现就是这样，
    实测在"障碍正挡在目标方向上"的场景会卡死在障碍前面。

``escape``（改进）
    目标方向被障碍挡住时，沿**障碍表面的切向**绕行（速度不为 0），并在持续
    卡住时翻转到另一侧；不在障碍区时与 baseline 完全一致。这样既保留原来的
    "朝目标直冲"手感，又能绕开挡路的障碍。
"""
from __future__ import annotations

import math

from car_2d_target_tracker.config import TrackerConfig
from car_2d_target_tracker.geometry import nearest_obstacle, normalize_angle


class Tracker:
    def __init__(self, cfg: TrackerConfig = None, mode: str = "escape"):
        if mode not in ("baseline", "escape"):
            raise ValueError(f"未知模式：{mode}")
        self.cfg = cfg or TrackerConfig()
        self.mode = mode
        self.reset()

    def reset(self):
        self.arrived = False
        self.side = +1.0          # 绕行方向：+1 逆时针、-1 顺时针
        self.flip_left = 0.0      # 剩余翻转时间
        self.stuck_acc = 0.0      # 连续"几乎没动"的时长
        self.escape_time = 0.0    # 已连续绕行时长

    # ---------------------------------------------------------------- 主逻辑
    def compute(self, car, target):
        """返回 ``(v, omega, arrived)``。"""
        cfg = self.cfg
        dx, dy = target.x - car.x, target.y - car.y
        dist = math.hypot(dx, dy)
        if dist < cfg.arrive_radius:
            self.arrived = True
            return 0.0, 0.0, True

        bearing = math.atan2(dy, dx)
        err = normalize_angle(bearing - car.theta)
        # 减速半径：baseline 沿原实现的 120 px；escape 用更小的 chase_taper，
        # 否则近距离速度被压到与目标同量级，追不上移动目标。
        taper = cfg.slow_radius if self.mode == "baseline" else cfg.chase_taper
        v = cfg.max_speed * max(0.0, math.cos(err)) * min(1.0, dist / taper)
        omega = max(-cfg.max_steer, min(cfg.max_steer, cfg.kp_turn * err))

        blocked = car.blocked_steps > getattr(self, "_seen_blocked", 0)
        self._seen_blocked = car.blocked_steps

        if not blocked:
            # 没被挡：和原实现一致
            self.escape_time = max(0.0, self.escape_time - cfg.dt)
            return v, omega, False

        # ---------- 被挡住的处理 ----------
        if self.mode == "baseline":
            # 原实现：停住 + 原地打转（靠打转慢慢蹭出去，容易卡死）
            return 0.0, math.copysign(cfg.max_steer, omega if omega else 1.0), False

        near = nearest_obstacle(car.x, car.y, cfg.obstacles)
        if near is None:
            return 0.0, omega, False
        ox, oy, r, _gap = near

        # 沿障碍表面切向：从"障碍心 → 车"的径向旋转 90°
        radial = math.atan2(car.y - oy, car.x - ox)
        tangent_a = radial + math.pi / 2.0
        tangent_b = radial - math.pi / 2.0

        # 卡住检测：连续一段时间几乎不动 → 翻转绕行方向
        self.stuck_acc = (self.stuck_acc + cfg.dt
                          if abs(car.v) < cfg.stuck_speed_eps else 0.0)
        if self.stuck_acc >= cfg.stuck_time and self.flip_left <= 0.0:
            self.side *= -1.0
            self.flip_left = cfg.stuck_flip_time
            self.stuck_acc = 0.0
        self.flip_left = max(0.0, self.flip_left - cfg.dt)

        # 选哪一侧：默认选与目标方位夹角更小的那条切向
        def angdiff(a):
            return abs(normalize_angle(a - bearing))

        if self.flip_left > 0.0:
            want = tangent_a if self.side > 0 else tangent_b
        else:
            want = tangent_a if angdiff(tangent_a) <= angdiff(tangent_b) else tangent_b

        err_t = normalize_angle(want - car.theta)
        omega_t = max(-cfg.max_steer, min(cfg.max_steer,
                                          cfg.kp_turn * err_t)) * cfg.escape_steer_ratio
        v_t = cfg.max_speed * cfg.escape_speed_ratio
        self.escape_time += cfg.dt
        return v_t, omega_t, False

    # ---------------------------------------------------------------- 工具
    def describe(self):
        return {"mode": self.mode, "side": self.side,
                "stuck_acc": self.stuck_acc, "escape_time": self.escape_time}
