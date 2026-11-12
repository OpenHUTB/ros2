#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Standoff reference generation and intercept planning.

Single-line summary: the follower holds a fixed distance behind the
moving target along its velocity direction at a fixed altitude; the
reference position/velocity given the target state is analytic, and the
intercept point is found by iterating time-to-intercept against the
target's own future positions.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: 参考速度一阶低通（s）——与仓库底座其它闭环一致，抑制参考速度阶跃
REF_TAU = 0.5
#: standoff 偏移方向的旋转限速（deg/s）：目标直角转弯时偏移方向
#: 会瞬间转 90°，参考点跳变 2·standoff·sin(Δ/2)≈17 m，位置环必然
#: 大幅超调；按角速度上限绕目标平滑绕行，参考点走圆弧而不是跳变。
OFFSET_RATE_DEG = 15.0
CLIMB_SPEED = 2.5          # m/s，垂直爬升速度
TAIL_S = 8.0               # 目标停车后的收尾悬停时长（s）
#: 截获判定半径：与 standoff 环的偏差进入该值即认为跟上了
CAPTURE_RADIUS_M = 2.5


def slew_dir(prev_dir2, desired_dir2, dt: float,
             rate_deg: float = OFFSET_RATE_DEG) -> np.ndarray:
    """把偏移方向按最大角速度 rate_deg 朝期望方向旋转一步。

    prev 与 desired 都是单位向量；两步之间的夹角超过 rate·dt 时只转
    rate·dt，保持旋转方向（叉积符号）。返回单位向量。
    """
    a = np.asarray(prev_dir2, dtype=float)[:2]
    b = np.asarray(desired_dir2, dtype=float)[:2]
    na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    if na < 1e-9:
        return b / max(nb, 1e-9)
    a = a / na
    if nb < 1e-9:
        return a
    b = b / nb
    cross = float(a[0] * b[1] - a[1] * b[0])
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    ang = float(np.arctan2(cross, dot))
    step = np.deg2rad(rate_deg) * dt
    if abs(ang) <= step:
        return b
    s = step if ang > 0.0 else -step
    c, sn = np.cos(s), np.sin(s)
    return np.array([a[0] * c - a[1] * sn, a[0] * sn + a[1] * c])


@dataclass(frozen=True)
class FollowCfg:
    """跟随几何参数。"""

    standoff_m: float = 12.0     # 目标后方水平距离（沿目标速度反方向）
    alt_m: float = 10.0          # 跟随高度
    cruise_speed: float = 9.0    # 无人机巡航速度上限（m/s）

    def __post_init__(self):
        if self.standoff_m <= 0.0 or self.alt_m <= 0.0:
            raise ValueError("standoff_m / alt_m must be positive")
        if self.cruise_speed <= 0.0:
            raise ValueError("cruise_speed must be positive")


def follow_reference(p_tgt2, v_tgt2, cfg: FollowCfg,
                     prev_dir2=None, dt: float | None = None) -> tuple:
    """由目标当前状态生成无人机参考 (p_ref3, v_ref3, offset_dir)。

    参考点 = 目标位置 + 目标速度反方向 × standoff_m（正后方），
    高度固定 alt_m；参考速度 = 目标速度（几何关系随目标平移）。
    目标静止（v≈0）时保持上次的偏移方向——直接取"当前速度反方向"
    会让参考点在停车瞬间跳到任意一侧；prev_dir2 为 None 则取 +x。
    给出 dt 时偏移方向按 OFFSET_RATE_DEG 限速旋转（转角处参考点
    绕目标走圆弧而不是 17 m 跳变）。

    返回的 offset_dir 是本次实际使用的单位偏移方向（节点与测试用来
    复核"参考点确实在目标正后方"）。
    """
    v = np.asarray(v_tgt2, dtype=float)[:2]
    speed = float(np.linalg.norm(v))
    if speed > 0.15:                       # 0.15 m/s 以下视为停车
        desired = -v / speed
    elif prev_dir2 is not None:
        desired = np.asarray(prev_dir2, dtype=float)[:2]
        n = float(np.linalg.norm(desired))
        desired = desired / n if n > 1e-9 else np.array([1.0, 0.0])
    else:
        desired = np.array([1.0, 0.0])
    v_ref = np.array([v[0], v[1], 0.0])
    if dt is not None and prev_dir2 is not None and speed > 0.15:
        d = slew_dir(prev_dir2, desired, dt)
        a = np.asarray(prev_dir2, dtype=float)[:2]
        a = a / max(float(np.linalg.norm(a)), 1e-9)
        cross = float(a[0] * desired[1] - a[1] * desired[0])
        dot = float(np.clip(np.dot(a, desired), -1.0, 1.0))
        if abs(float(np.arctan2(cross, dot))) > np.deg2rad(OFFSET_RATE_DEG) * dt:
            # 正在限速旋转：参考点沿 standoff 圆弧切向移动，把切向线速度
            # 计入参考速度（ω×r），否则位置环要独自追这段弧
            omega = np.deg2rad(OFFSET_RATE_DEG) * (1.0 if cross >= 0.0 else -1.0)
            tang = np.array([-d[1], d[0]])
            v_ref[:2] += tang * omega * cfg.standoff_m
    else:
        d = desired
    p_t = np.asarray(p_tgt2, dtype=float)[:2]
    p_ref = np.array([p_t[0] + d[0] * cfg.standoff_m,
                      p_t[1] + d[1] * cfg.standoff_m, cfg.alt_m])
    return p_ref, v_ref, d


def plan_intercept(target, start_xy, cfg: FollowCfg,
                   t_max: float | None = None) -> dict:
    """规划截获：找到无人机能追上 standoff 环的最早时刻。

    无人机路线：从 start_xy 垂直爬升到 alt_m，再以巡航速度直飞
    t 时刻的参考点。固定点迭代：t_avail(t) = 爬升时长 +
    |start_xy − ref_xy(t)| / cruise_speed，与 t 相等即截获。
    t_max 内追不上（目标速度接近巡航上限或路径太远）返回 feasible=False
    与建议（提高巡航速度或缩小 standoff）。
    """
    climb_s = cfg.alt_m / CLIMB_SPEED
    t0 = climb_s                     # 最早可能到达时刻
    t_hi = t_max if t_max is not None else target.duration_s + 120.0
    p0 = np.asarray(start_xy, dtype=float)[:2]

    def t_avail(t: float) -> float:
        p_ref, _v, _d = follow_reference(*target.pos_vel(min(t, target.duration_s)), cfg)
        dist = float(np.linalg.norm(p_ref[:2] - p0))
        return climb_s + dist / cfg.cruise_speed

    # f(t) = t_avail(t) − t：爬升段 f>0（还没起飞完），随 t 增大穿越零点
    t = max(t0, 0.0)
    feasible = False
    for _ in range(60):
        f = t_avail(t) - t
        if abs(f) < 0.05:
            feasible = True
            break
        if f > 0.0:
            t = min(t + max(0.5, f), t_hi)
        else:
            t = max(t - min(0.5, -f), t0)
        if t >= t_hi - 1e-9:
            break
    if not feasible:
        f_end = t_avail(target.duration_s) - target.duration_s
        return dict(feasible=False, t_intercept_s=None,
                    advice="cannot catch up: raise cruise speed to "
                           ">= %.1f m/s or reduce standoff"
                           % (target.speed * 1.2))
    p_ref, v_ref, d = follow_reference(*target.pos_vel(min(t, target.duration_s)), cfg)
    return dict(feasible=True, t_intercept_s=round(float(t), 2),
                intercept_xy=[round(float(p_ref[0]), 2),
                              round(float(p_ref[1]), 2)],
                climb_s=round(climb_s, 2))
