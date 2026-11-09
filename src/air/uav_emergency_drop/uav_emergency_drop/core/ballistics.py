#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Flat-release ballistics: analytic solution + wind drift + scatter.

Single-line summary: analytic projectile solution for supply drops from
altitude h at speed v, with wind drift (0.8 factor) and empirical scatter
sigma = 0.15*h.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class BallisticsCfg:
    """drop 物理参数（全模块共用一份默认值）。"""

    g: float = 9.81            # m/s^2
    wind_coeff: float = 0.8    # 风偏经验系数
    sigma_factor: float = 0.15  # 落点散布 σ = factor * h
    accept_radius: float = 6.0  # 判定投放区间的可接受落点半径（m）


@dataclass(frozen=True)
class ReleasePlan:
    """一组投放条件（h, v, wind）下的解析量。"""

    h: float
    speed: float
    wind: tuple
    t_fall: float
    lead_m: float       # 沿航迹前抛距离 = v * t_fall
    drift: np.ndarray   # 风偏水平向量（m）
    sigma: float        # 落点散布半径（m）

    def as_dict(self) -> dict:
        return dict(h_m=self.h, speed_mps=self.speed,
                    wind_mps=[round(float(self.wind[0]), 2),
                              round(float(self.wind[1]), 2)],
                    t_fall_s=round(self.t_fall, 3),
                    lead_m=round(self.lead_m, 2),
                    drift_m=[round(float(self.drift[0]), 2),
                             round(float(self.drift[1]), 2)],
                    sigma_m=round(self.sigma, 2))


def fall_time(h: float, cfg: BallisticsCfg = BallisticsCfg()) -> float:
    """t_fall = sqrt(2h/g)。"""
    if h <= 0.0:
        raise ValueError("drop height must be positive, got %r" % h)
    return math.sqrt(2.0 * h / cfg.g)


def scatter_sigma(h: float, cfg: BallisticsCfg = BallisticsCfg()) -> float:
    """经验散布半径 σ = 0.15 h。"""
    return cfg.sigma_factor * h


def wind_drift(wind_xy, h: float, cfg: BallisticsCfg = BallisticsCfg()) -> np.ndarray:
    """落点风偏 = w · t_fall · 0.8。"""
    w = np.asarray(wind_xy, dtype=float)
    return w * fall_time(h, cfg) * cfg.wind_coeff


def make_release_plan(h: float, speed: float, wind_xy=(0.0, 0.0),
                      cfg: BallisticsCfg = BallisticsCfg()) -> ReleasePlan:
    """由投放条件构造解析解对象。"""
    t = fall_time(h, cfg)
    return ReleasePlan(h=float(h), speed=float(speed), wind=tuple(float(x) for x in wind_xy),
                       t_fall=t, lead_m=float(speed) * t,
                       drift=wind_drift(wind_xy, h, cfg),
                       sigma=scatter_sigma(h, cfg))


def impact_point(pos_xy, vel_xy, plan: ReleasePlan) -> np.ndarray:
    """由释放时刻位置/速度预测落点：p + v·t_fall + wind_drift。"""
    p = np.asarray(pos_xy, dtype=float)
    v = np.asarray(vel_xy, dtype=float)
    return p + v * plan.t_fall + plan.drift


def fall_time_from_state(z_rel: float, vz: float,
                         cfg: BallisticsCfg = BallisticsCfg()) -> float:
    """由实际离地高度与竖直速度解下落时间。

    z 轴向上、重力向下：z(t) = z0 + vz·t − g·t²/2，令 z = 0 取正根。
    vz 向上会显著拉长下落时间（落点随之前移），vz 向下则缩短；模型
    仍按同一无阻力假设外推，落点偏差交给调用方用接受半径校验。
    z_rel > 0 时判别式恒正，必可达地面。
    """
    if z_rel <= 0.0:
        raise ValueError("state at/below ground: z_rel=%r" % z_rel)
    return (vz + math.sqrt(vz * vz + 2.0 * cfg.g * z_rel)) / cfg.g


def impact_point_state(pos_3d_rel, vel_3d, wind_xy=(0.0, 0.0),
                       cfg: BallisticsCfg = BallisticsCfg()) -> np.ndarray:
    """由真实三维状态预测落点（实际高度与竖直速度参与解算）。

    pos/vel 为 (x, y, z) 三维，z 以地面为基准（调用方先减去地面高度）。
    落点 = 水平位置 + 水平速度·t + 风速·t·wind_coeff，t 由
    fall_time_from_state 解出——区别于 impact_point 用计划高度的
    t_fall：真实释放状态偏离计划（掉高、竖直速度不为零）时落点会
    随之偏移，偏差由调用方对照接受半径判定。
    """
    p = np.asarray(pos_3d_rel, dtype=float)
    v = np.asarray(vel_3d, dtype=float)
    t = fall_time_from_state(float(p[2]), float(v[2]), cfg)
    w = np.asarray(wind_xy, dtype=float)
    return p[:2] + v[:2] * t + w * t * cfg.wind_coeff


def aim_release_point(target, vel_xy, plan: ReleasePlan) -> np.ndarray:
    """理想释放点：target − v·t_fall − drift（风偏反推）。"""
    t = np.asarray(target, dtype=float)
    v = np.asarray(vel_xy, dtype=float)
    return t - v * plan.t_fall - plan.drift


def overflight_target(target, plan: ReleasePlan) -> np.ndarray:
    """风补偿后的越顶点：航线应经过 target − drift（逆风偏移）。"""
    return np.asarray(target, dtype=float) - plan.drift


@dataclass(frozen=True)
class ReleaseWindow:
    """一段直飞航线上可行的释放位置区间（沿航迹弧长坐标）。"""

    feasible: bool
    s_enter: float          # 区间起点（距段起点弧长，m）
    s_exit: float           # 区间终点
    s_ideal: float          # 理想释放点弧长
    lead_m: float           # 沿航迹前抛距离
    cross_error_m: float    # 理想释放点到航线的横向偏差
    advice: str = ""

    def as_dict(self) -> dict:
        return dict(feasible=self.feasible, s_enter_m=round(self.s_enter, 2),
                    s_exit_m=round(self.s_exit, 2), s_ideal_m=round(self.s_ideal, 2),
                    lead_m=round(self.lead_m, 2),
                    cross_error_m=round(self.cross_error_m, 2),
                    advice=self.advice)


def release_window(seg_a, seg_b, target, plan: ReleasePlan,
                   cfg: BallisticsCfg = BallisticsCfg()) -> ReleaseWindow:
    """解析求解“提前量对应的位置区间”。

    理想释放点 p* = target − v·t_fall − drift，把它投影到段方向上得到
    理想弧长 s_ideal；落点误差 ≤ accept_radius 对应沿迹半宽
    sqrt(R² − cross²)。区间完全落在段内才可直接投放，否则给出降速/
    绕行建议。
    """
    a = np.asarray(seg_a, dtype=float)
    b = np.asarray(seg_b, dtype=float)
    d = b - a
    seg_len = float(np.linalg.norm(d))
    if seg_len < 1e-9:
        return ReleaseWindow(False, 0.0, 0.0, 0.0, plan.lead_m, math.inf,
                             "degenerate segment")
    u = d / seg_len
    vel_xy = plan.speed * u
    p_star = aim_release_point(target, vel_xy, plan)
    q = p_star - a
    s_ideal = float(np.dot(q, u))
    cross = float(np.linalg.norm(q - s_ideal * u))
    r = cfg.accept_radius
    half = math.sqrt(max(0.0, r * r - cross * cross))
    s_enter, s_exit = s_ideal - half, s_ideal + half

    advice = ""
    feasible = True
    if cross > r:
        feasible = False
        advice = ("approach line off by %.1f m laterally; realign route or "
                  "use wind-compensated overflight point" % cross)
    elif s_enter < 0.0 or s_exit > seg_len:
        feasible = False
        v_max = seg_len / plan.t_fall
        if s_exit > seg_len:
            advice = ("lead %.1f m exceeds remaining segment; slow to "
                      "<= %.1f m/s or take a longer detour" % (plan.lead_m, v_max))
        else:
            advice = ("release window starts %.1f m before segment entry; "
                      "extend approach leg or slow to <= %.1f m/s"
                      % (-s_enter, v_max))
    return ReleaseWindow(feasible, s_enter, s_exit, s_ideal,
                         plan.lead_m, cross, advice)


def conditions_table(cfg: BallisticsCfg = BallisticsCfg()) -> list:
    """h ∈ {20,30,50} × v ∈ {6,8,10} 无风条件下的提前量与散布表。"""
    rows = []
    for h in (20, 30, 50):
        for v in (6, 8, 10):
            plan = make_release_plan(h, v, (0.0, 0.0), cfg)
            rows.append(dict(h_m=h, speed_mps=v,
                             t_fall_s=round(plan.t_fall, 3),
                             lead_m=round(plan.lead_m, 2),
                             sigma_m=round(plan.sigma, 2)))
    return rows
