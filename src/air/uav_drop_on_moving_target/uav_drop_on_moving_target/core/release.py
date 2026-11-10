#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""投放时刻解算：让"货落地"与"车到点"发生在同一时刻同一位置。

一行摘要：给定无人机标称航迹与车辆未来轨迹，扫出使落点误差最小的释放
时刻；两条对照基线分别回答"瞄准车辆当前位置会差多少"和"把中途停车当
没发生会差多少"。
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Optional, Tuple

import numpy as np

from .ballistics import (BallisticsCfg, fall_time, fall_time_from_state,
                         impact_point, scatter_sigma, wind_drift)
from .track import PolylineTrack
from .vehicle import Vehicle


@dataclass(frozen=True)
class DropCfg:
    """一次投放任务的条件：高度、无人机速度、风、接受半径。"""

    alt_m: float = 30.0           # 投放高度（离地）
    speed: float = 8.0            # 无人机沿航迹巡航速度（标称解算用）
    accept_radius_m: float = 6.0  # 落点允许偏差
    wind: Tuple[float, float] = (0.0, 0.0)
    scan_dt: float = 0.05         # 粗扫步长（s）
    horizon_s: float = 240.0      # 解算时间上限（s）
    climb_speed: float = 2.5      # 垂直爬升速度（s 起点对齐闭环的爬升段）
    #: 释放瞬间无人机的最低头速：投放动作在飞行中完成，机体停住时
    #: "投放解"没有意义（航迹末端悬停会让解算退化成"停在一处等车开过来"）
    min_uav_speed_mps: float = 0.5
    ball: BallisticsCfg = BallisticsCfg()

    def __post_init__(self):
        if not np.isfinite(self.alt_m) or self.alt_m <= 0.0:
            raise ValueError("alt_m must be positive and finite")
        if not np.isfinite(self.speed) or self.speed <= 0.0:
            raise ValueError("speed must be positive and finite")
        if not np.isfinite(self.accept_radius_m) or self.accept_radius_m <= 0.0:
            raise ValueError("accept_radius_m must be positive and finite")
        w = np.asarray(self.wind, dtype=float)
        if w.shape != (2,) or not np.all(np.isfinite(w)):
            raise ValueError("wind must be a finite 2-vector")
        if not np.isfinite(self.scan_dt) or self.scan_dt <= 0.0:
            raise ValueError("scan_dt must be positive and finite")

    @property
    def climb_s(self) -> float:
        """起飞爬升到投放高度的耗时：闭环在这段时间里还没进入航迹。"""
        return self.alt_m / self.climb_speed

    @property
    def t_fall(self) -> float:
        """静止平飞投放的下落时间。"""
        return fall_time(self.alt_m, self.ball)

    @property
    def drift(self) -> np.ndarray:
        """这一组条件下的落点风偏向量（m）。"""
        return wind_drift(self.wind, self.t_fall, self.ball)


@dataclass
class ReleaseSolution:
    """一次解算的结果（含两条对照基线，便于直接写进结果表）。"""

    feasible: bool
    t_release: Optional[float]
    release_xy: Optional[np.ndarray]
    aim_xy: Optional[np.ndarray]
    impact_xy: Optional[np.ndarray]
    vehicle_at_impact: Optional[np.ndarray]
    miss_m: Optional[float]
    min_miss_m: float                  # 全时段最小误差（不可行时也有值）
    cross_error_m: Optional[float]     # 瞄准点相对航迹的横向偏差
    t_fall: float
    naive_t_release: Optional[float]   # 瞄准"车辆当前位置"的释放时刻
    naive_miss_m: Optional[float]      # 该策略下的落点误差
    uniform_miss_m: Optional[float]    # 忽略中途停车（按匀速规划）的误差
    tangent_miss_m: Optional[float]    # 按释放瞬间速度直线外推的误差
    advice: str = ""

    def as_dict(self) -> dict:
        def r2(v):
            return None if v is None else [round(float(v[0]), 2),
                                           round(float(v[1]), 2)]
        return dict(
            feasible=bool(self.feasible),
            t_release_s=None if self.t_release is None else round(self.t_release, 2),
            release_m=r2(self.release_xy),
            aim_m=r2(self.aim_xy),
            impact_m=r2(self.impact_xy),
            vehicle_at_impact_m=r2(self.vehicle_at_impact),
            miss_m=None if self.miss_m is None else round(self.miss_m, 2),
            min_miss_m=None if not np.isfinite(self.min_miss_m)
            else round(self.min_miss_m, 2),
            cross_error_m=None if self.cross_error_m is None
            else round(self.cross_error_m, 2),
            t_fall_s=round(self.t_fall, 3),
            naive_t_release_s=None if self.naive_t_release is None
            else round(self.naive_t_release, 2),
            naive_miss_m=None if self.naive_miss_m is None
            else round(self.naive_miss_m, 2),
            uniform_miss_m=None if self.uniform_miss_m is None
            else round(self.uniform_miss_m, 2),
            tangent_miss_m=None if self.tangent_miss_m is None
            else round(self.tangent_miss_m, 2),
            advice=self.advice)


def _uav_state_at(track: PolylineTrack, t: float, cfg: DropCfg):
    """绝对时刻 t 的无人机标称状态（t < climb_s 时在起点上方爬升）。"""
    if t < cfg.climb_s:
        start = np.asarray(track.points[0], dtype=float)
        return (np.array([start[0], start[1], cfg.alt_m * t / cfg.climb_s]),
                np.array([0.0, 0.0, cfg.climb_speed]))
    p2, v2 = track.pos_vel(t - cfg.climb_s)
    return (np.array([p2[0], p2[1], cfg.alt_m], dtype=float),
            np.array([v2[0], v2[1], 0.0], dtype=float))


def miss_at(track: PolylineTrack, vehicle: Vehicle, cfg: DropCfg, t: float) -> float:
    """标称航迹 t 时刻释放的落点误差；任一方停住时记 inf。"""
    p3, v3 = _uav_state_at(track, t, cfg)
    if float(np.linalg.norm(v3[:2])) < cfg.min_uav_speed_mps:
        return float("inf")
    imp = impact_point(p3, v3, cfg.wind, cfg.ball)
    return float(np.linalg.norm(imp - vehicle.pos_at(t + cfg.t_fall)))


def best_release_time(track: PolylineTrack, vehicle: Vehicle,
                      cfg: DropCfg) -> Tuple[float, float]:
    """粗扫 + 细扫给出 (最优释放时刻, 该时刻误差)。"""
    grid = np.arange(0.0, cfg.horizon_s + 1e-9, cfg.scan_dt)
    misses = np.array([miss_at(track, vehicle, cfg, float(t)) for t in grid])
    if np.all(np.isinf(misses)):
        return 0.0, float("inf")
    i = int(np.argmin(misses))
    lo = max(0.0, float(grid[i]) - 3.0 * cfg.scan_dt)
    hi = min(cfg.horizon_s, float(grid[i]) + 3.0 * cfg.scan_dt)
    fine = np.arange(lo, hi + 1e-12, 0.002)
    fine_miss = np.array([miss_at(track, vehicle, cfg, float(t)) for t in fine])
    j = int(np.argmin(fine_miss))
    return float(fine[j]), float(fine_miss[j])


def solve_release(track: PolylineTrack, vehicle: Vehicle,
                  cfg: DropCfg = DropCfg()) -> ReleaseSolution:
    """解出释放时刻，并给出两条基线的误差。"""
    t_r, miss = best_release_time(track, vehicle, cfg)
    t_naive, naive_miss = _naive_solution(track, vehicle, cfg)
    uniform_miss = _uniform_miss(track, vehicle, cfg)

    if not np.isfinite(miss) or miss > cfg.accept_radius_m:
        p3, _ = _uav_state_at(track, t_r, cfg)
        return ReleaseSolution(
            feasible=False, t_release=None, release_xy=None, aim_xy=None,
            impact_xy=None, vehicle_at_impact=None, miss_m=None,
            min_miss_m=float(miss), cross_error_m=None, t_fall=cfg.t_fall,
            naive_t_release=t_naive, naive_miss_m=naive_miss,
            uniform_miss_m=uniform_miss, tangent_miss_m=None,
            advice=_advice(vehicle, cfg, t_r, miss, p3))

    p3, v3 = _uav_state_at(track, t_r, cfg)
    imp = impact_point(p3, v3, cfg.wind, cfg.ball)
    tgt = vehicle.pos_at(t_r + cfg.t_fall)
    aim = tgt - cfg.drift
    speed = float(np.linalg.norm(v3[:2]))
    heading = v3[:2] / max(speed, 1e-9)
    rel = aim - p3[:2]
    cross = float(abs(rel[0] * (-heading[1]) + rel[1] * heading[0]))
    # 基线三：把车当成"保持当前速度方向直线走"外推（忽略前方转角）
    v_now = vehicle.pos_vel(t_r)[1]
    tangent = float(np.linalg.norm(
        vehicle.pos_at(t_r) + v_now * cfg.t_fall - tgt))
    return ReleaseSolution(
        feasible=True, t_release=t_r, release_xy=p3[:2].copy(), aim_xy=aim,
        impact_xy=imp, vehicle_at_impact=tgt, miss_m=float(miss),
        min_miss_m=float(miss), cross_error_m=cross, t_fall=cfg.t_fall,
        naive_t_release=t_naive, naive_miss_m=naive_miss,
        uniform_miss_m=uniform_miss, tangent_miss_m=tangent, advice="")


def _advice(vehicle: Vehicle, cfg: DropCfg, t_best: float, miss: float,
            release_xy: np.ndarray) -> str:
    """不可行时给出具体可执行的建议。"""
    if not np.isfinite(miss):
        return ("no release point while the aircraft and the convoy are both "
                "moving: the route never brings the aircraft into the lead "
                "position")
    veh_speed = vehicle.speed_at(t_best + cfg.t_fall)
    if miss <= cfg.accept_radius_m * 1.5:
        return ("closest approach %.1f m at t=%.1f s, just outside the %.1f m "
                "radius: tighten the approach line or accept a wider radius"
                % (miss, t_best, cfg.accept_radius_m))
    if veh_speed > 1e-6:
        return ("convoy moves %.1f m during the %.2f s fall; on this route the "
                "aircraft only gets within %.1f m — extend the approach leg or "
                "raise cruise speed above %.1f m/s"
                % (veh_speed * cfg.t_fall, cfg.t_fall, miss, vehicle.speed))
    return ("convoy is parked near (%.0f, %.0f) but the route never comes "
            "within %.1f m of it (min miss %.1f m)"
            % (release_xy[0], release_xy[1], cfg.accept_radius_m, miss))


def _naive_solution(track: PolylineTrack, vehicle: Vehicle, cfg: DropCfg):
    """基线一：把货瞄准车辆"此刻"所在位置。

    求解 uav(t) + v_uav·t_fall + 风偏 = veh(t) 得到下键时刻；货落到
    veh(t)，而车已经走到 veh(t + t_fall)，差值就是这条策略的误差——
    它约等于车速 × 下落时间，与无人机怎么飞无关。
    """
    grid = np.arange(0.0, cfg.horizon_s + 1e-9, cfg.scan_dt)
    gaps = np.full(grid.shape, np.inf)
    for k, t in enumerate(grid):
        p3, v3 = _uav_state_at(track, float(t), cfg)
        if float(np.linalg.norm(v3[:2])) < cfg.min_uav_speed_mps:
            continue
        imp = impact_point(p3, v3, cfg.wind, cfg.ball)
        gaps[k] = float(np.linalg.norm(imp - vehicle.pos_at(float(t))))
    if np.all(np.isinf(gaps)):
        return None, None
    t_n = float(grid[int(np.argmin(gaps))])
    p3, v3 = _uav_state_at(track, t_n, cfg)
    imp = impact_point(p3, v3, cfg.wind, cfg.ball)
    return t_n, float(np.linalg.norm(imp - vehicle.pos_at(t_n + cfg.t_fall)))


def _uniform_miss(track: PolylineTrack, vehicle: Vehicle, cfg: DropCfg):
    """基线二：规划时把"中途停车"当没发生（只按总里程/车速算到达时刻）。

    只有带停留的车辆才有这条基线；没有停留时返回 None。
    """
    if not vehicle.dwell_s:
        return None
    plain = replace(vehicle, dwell_s=())
    t_u, _m = best_release_time(track, plain, cfg)
    if not np.isfinite(t_u):
        return None
    p3, v3 = _uav_state_at(track, t_u, cfg)
    imp = impact_point(p3, v3, cfg.wind, cfg.ball)
    return float(np.linalg.norm(imp - vehicle.pos_at(t_u + cfg.t_fall)))


def evaluate_release(state_3d, vel_3d, t: float, vehicle: Vehicle,
                     cfg: DropCfg = DropCfg()) -> dict:
    """用真实状态判定"现在松手能不能投中"（闭环与 ROS 节点共用这条判据）。

    与 solve_release 的区别：不假设无人机在标称航迹上，位置/速度都来自
    真实状态；高度与竖直速度参与解下落时间，掉高、抬头释放都会反映到
    落点误差上。
    """
    p = np.asarray(state_3d, dtype=float)
    v = np.asarray(vel_3d, dtype=float)
    if p.size != 3 or v.size != 3:
        return dict(ok=False, reason="state/velocity must be 3-D",
                    miss_m=None, impact_xy=None, target_xy=None, t_fall=None)
    if not np.all(np.isfinite(p)) or not np.all(np.isfinite(v)):
        return dict(ok=False, reason="non-finite state", miss_m=None,
                    impact_xy=None, target_xy=None, t_fall=None)
    if float(p[2]) <= 0.0:
        return dict(ok=False, reason="state at/below ground: z=%.2f" % p[2],
                    miss_m=None, impact_xy=None, target_xy=None, t_fall=None)
    try:
        t_fall = fall_time_from_state(float(p[2]), float(v[2]), cfg.ball)
    except ValueError:
        return dict(ok=False, reason="invalid vertical state",
                    miss_m=None, impact_xy=None, target_xy=None, t_fall=None)
    imp = p[:2] + v[:2] * t_fall + wind_drift(cfg.wind, t_fall, cfg.ball)
    tgt = vehicle.pos_at(t + t_fall)
    miss = float(np.linalg.norm(imp - tgt))
    ok = miss <= cfg.accept_radius_m
    return dict(ok=ok, reason="" if ok else "impact %.2f m off target" % miss,
                miss_m=miss, impact_xy=imp, target_xy=tgt, t_fall=t_fall)


def scatter_radius(cfg: DropCfg = DropCfg()) -> float:
    """当前高度下的落点散布半径（工程近似 σ = 0.15 h）。"""
    return scatter_sigma(cfg.alt_m, cfg.ball)
