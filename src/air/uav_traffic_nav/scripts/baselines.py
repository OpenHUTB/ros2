#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对照基线控制器（评测用，非训练策略）.

两个基线都直接读取环境真值（``env.pos`` / ``env.cars``），属于**特权信息**基线，
因此它们的表现是"信息更多、但没有学习"的下界/参照，而不是公平的同信息对比：

* ``greedy``：只朝目标飞，完全无视车辆 —— 用来证明"不避障就会撞"；
* ``vo``：速度障碍法，采样候选速度，剔除 1.2 s 内会与车辆相撞的方向，
  在安全候选里选最偏向目标的一档 —— 经典几何避障，作为无学习对照。
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from policy import (MAX_HORIZ_SPEED, time_to_collision, velocity_to_action)

# 车辆半径 + 无人机半径，给几何避障留一点余量
_SAFE_RADIUS = 1.2 + 0.4 + 0.3
_LOOKAHEAD = 1.2


def _unit_to_goal(env) -> Tuple[float, float]:
    dx = float(env.goal[0] - env.pos[0])
    dy = float(env.goal[1] - env.pos[1])
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return 1.0, 0.0
    return dx / norm, dy / norm


def greedy_action(env) -> np.ndarray:
    """径直朝目标，最大水平速度。"""
    ux, uy = _unit_to_goal(env)
    return velocity_to_action(ux * MAX_HORIZ_SPEED, uy * MAX_HORIZ_SPEED)


def _candidate_blocked(px, py, vx, vy, cars: Sequence[dict]) -> bool:
    """预测 _LOOKAHEAD 秒内的直线运动是否与任一车辆相交（圆-线段距离）."""
    ex, ey = px + vx * _LOOKAHEAD, py + vy * _LOOKAHEAD
    for car in cars:
        cx, cy = car["x"] + car["vx"] * _LOOKAHEAD, car["y"] + car["vy"] * _LOOKAHEAD
        # 车辆也会动：用相对运动的最近点近似（预测终点处按车自己的位移修正）
        mx, my = cx - ex, cy - ey
        # 线段 (px,py)->(ex,ey) 到点 (cx,cy) 的距离
        sx, sy = ex - px, ey - py
        seg = sx * sx + sy * sy
        if seg < 1e-9:
            dist = math.hypot(cx - px, cy - py)
        else:
            t = ((cx - px) * sx + (cy - py) * sy) / seg
            t = max(0.0, min(1.0, t))
            dist = math.hypot(cx - (px + t * sx), cy - (py + t * sy))
        if dist < _SAFE_RADIUS or math.hypot(mx, my) < _SAFE_RADIUS:
            return True
    return False


def vo_action(env, n_dirs: int = 16) -> np.ndarray:
    """速度障碍采样：先试全速，再试半速；都不安全就原地悬停."""
    ux, uy = _unit_to_goal(env)
    traffic = [{"x": c["x"], "y": c["y"], "vx": c["vx"], "vy": c["vy"]} for c in env.cars]
    ttc_risk = any(
        time_to_collision((env.pos[0], env.pos[1]), (env.vel[0], env.vel[1]),
                          (c["x"], c["y"]), (c["vx"], c["vy"]), max_time=_LOOKAHEAD) is not None
        for c in env.cars)

    best: Optional[Tuple[float, float, float]] = None
    for speed in (MAX_HORIZ_SPEED, MAX_HORIZ_SPEED * 0.5):
        for i in range(n_dirs):
            if speed == MAX_HORIZ_SPEED and i == 0:
                vx, vy = ux * speed, uy * speed      # 优先直冲目标
            else:
                ang = 2.0 * math.pi * i / n_dirs
                vx, vy = math.cos(ang) * speed, math.sin(ang) * speed
            if _candidate_blocked(env.pos[0], env.pos[1], vx, vy, traffic):
                continue
            score = (vx * ux + vy * uy) - 0.05 * abs(math.atan2(vy, vx) - math.atan2(uy, ux))
            if best is None or score > best[0]:
                best = (score, vx, vy)
        if best is not None:
            break

    if best is None:
        # 已经没有安全候选：若当前有 TTC 冲突就停，否则保持朝目标（交给环境判定）
        if ttc_risk:
            return velocity_to_action(0.0, 0.0)
        return velocity_to_action(ux * MAX_HORIZ_SPEED * 0.3, uy * MAX_HORIZ_SPEED * 0.3)
    return velocity_to_action(best[1], best[2])


BASELINES = {"greedy": greedy_action, "vo": vo_action}


# ---------------------------------------------------------------------------
# 规则专家：用于 DAgger / 行为克隆的示范来源（评测时也可作为上界参照）
# ---------------------------------------------------------------------------
def _lane_conflict(env, lane, window):
    """是否有车会在 window 秒内占用本机 x 附近的穿越走廊."""
    px = float(env.pos[0])
    for car in env.cars:
        if abs(float(car["y"]) - lane) > 0.6:
            continue
        vx = float(car["vx"])
        if abs(vx) < 1e-6:
            continue
        cx = float(car["x"])
        half = 1.3
        t_enter, t_exit = (px - half - cx) / vx, (px + half - cx) / vx
        if t_enter > t_exit:
            t_enter, t_exit = t_exit, t_enter
        if t_exit < -0.1 or t_enter > window:
            continue
        return True
    return False


def expert_action(env, wait_distance=3.0, window=1.2, commit_distance=1.35):
    """规则专家：在车道前 ``wait_distance`` 米处等窗口，确认没有车经过再穿过去.

    无状态（只看当前状态），因此可以直接用于行为克隆；
    读取全部真值，只作示范/上界参照，不是公平基线。
    """
    px, py = float(env.pos[0]), float(env.pos[1])
    gx, gy = float(env.goal[0]), float(env.goal[1])
    lanes = [y for y in env.lane_ys if (py - y) * (gy - y) <= 0.0]
    if lanes:
        lane = min(lanes, key=lambda y: abs(y - py))
        sign = 1.0 if py > lane else -1.0
        dist = abs(py - lane)
        approaching = (py - lane) * sign > 0.0
        # 只在"还没进入危险走廊"时等待；一旦进入就保持穿越，避免卡在车道里
        if approaching and commit_distance < dist <= wait_distance:
            if _lane_conflict(env, lane, window):
                return velocity_to_action(0.0, 0.0)
    ux, uy = gx - px, gy - py
    n = math.hypot(ux, uy) or 1.0
    return velocity_to_action(ux / n * MAX_HORIZ_SPEED, uy / n * MAX_HORIZ_SPEED)