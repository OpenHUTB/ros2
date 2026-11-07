#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo closed-loop flyer for the first N legs of a route.

取航线前 N 个航点段（改进前后各一份）在 uav_mujoco_sim 底座上真飞：
位置目标沿分段线性航迹推进，观察点处悬停 ``hover_time`` 秒拍摄；
参考速度前馈经一阶低通（tau=0.5 s），姿态阻尼 kw=-0.18 抑制航向阶跃
激励。统计量全部来自刚体积分出的真实位置，而不是计划值。
"""
from __future__ import annotations

import os
import sys
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Set, Tuple

import numpy as np

from .geometry import Params, Waypoint, polyline_length

#: 姿态阻尼（与参考实现同款写法，抑制分段航迹激励出的振荡）
ATT_DAMPING = (-0.18, -0.18, -0.126)
TAIL_S = 6.0   # 末端保持时长：转角超调（约 20~30 m）需要足够时间收敛到终点


def _load_backend():
    """导入课程仿真底座；源码目录直接运行时自动挂同级包路径。"""
    try:
        from uav_mujoco_sim.core.quadrotor import (  # type: ignore
            GeometricController, Quadrotor)
        from uav_mujoco_sim.core.paths import models_dir  # type: ignore
        return Quadrotor, GeometricController, models_dir()
    except Exception:
        here = Path(__file__).resolve()
        sibling = here.parents[3] / "uav_mujoco_sim"
        if sibling.is_dir() and str(sibling) not in sys.path:
            sys.path.insert(0, str(sibling))
        from uav_mujoco_sim.core.quadrotor import (  # type: ignore
            GeometricController, Quadrotor)
        from uav_mujoco_sim.core.paths import models_dir  # type: ignore
        return Quadrotor, GeometricController, models_dir()


@dataclass
class FlyStats:
    """一次闭环真飞的统计（真实位置积分）。"""

    legs: int
    planned_m: float          # 前 N 段计划航程
    actual_m: float           # 真飞位置积分航程
    arrival_error_m: float    # 终点位置误差（悬停收敛后）
    max_track_error_m: float  # 全程最大跟踪误差
    sim_time_s: float
    wall_time_s: float

    def as_dict(self) -> dict:
        return dict(legs=self.legs,
                    planned_m=round(self.planned_m, 1),
                    actual_m=round(self.actual_m, 1),
                    arrival_error_m=round(self.arrival_error_m, 2),
                    max_track_error_m=round(self.max_track_error_m, 2),
                    sim_time_s=round(self.sim_time_s, 1),
                    wall_time_s=round(self.wall_time_s, 2))


def _segment_speeds(path: Sequence[Waypoint], hover_at: Set[int],
                    params: Params) -> List[float]:
    """每段巡航速度（speeds[k] 为第 k-1→k 段）：到达点转角越大越减速，
    起步 / 悬停 / 终点减速进入（可飞性约束）。"""
    n = len(path)
    cruise = params.cruise_speed
    speeds = [0.0]
    for k in range(1, n):
        d_in = np.asarray(path[k]) - np.asarray(path[k - 1])
        f = 0.5 if (k == 1 or k in hover_at or k == n - 1) else 1.0
        if 1 < k < n - 1:
            d_out = np.asarray(path[k + 1]) - np.asarray(path[k])
            cosv = float(np.clip(
                d_in @ d_out / (np.linalg.norm(d_in) * np.linalg.norm(d_out)
                                + 1e-9), -1.0, 1.0))
            theta = math.acos(cosv)
            if theta > 0.1:
                f = min(f, 1.0 - 0.7 * theta / math.pi)
        speeds.append(max(0.25, f) * cruise)
    return speeds


def build_timeline(path: Sequence[Waypoint], hover_at: Set[int],
                   params: Params) -> Tuple[List[float], List[float]]:
    """时间参数化：各段按转角减速巡航，悬停航点停留 hover_time 秒。

    返回每个航点的到达时刻与悬停结束时刻（非悬停点两者相同）。
    """
    speeds = _segment_speeds(path, hover_at, params)
    arrive, t = [0.0], 0.0
    for i in range(1, len(path)):
        d = float(np.linalg.norm(np.asarray(path[i]) - np.asarray(path[i - 1])))
        t += d / speeds[i]
        arrive.append(t)
        if i in hover_at:
            t += params.hover_time
    return arrive, [a + (params.hover_time if i in hover_at else 0.0)
                    for i, a in enumerate(arrive)]


def _reference(path: Sequence[Waypoint], arrive: Sequence[float],
               leave: Sequence[float], t: float) -> Tuple[np.ndarray, np.ndarray]:
    """t 时刻的参考位置与速度：段内匀速、悬停点静止、末端保持。"""
    n = len(path) - 1
    for i in range(1, n + 1):
        if t < arrive[i] - 1e-9:               # 正在 i-1 → i 巡航
            a = np.asarray(path[i - 1], dtype=float)
            b = np.asarray(path[i], dtype=float)
            seg = max(arrive[i] - leave[i - 1], 1e-9)
            u = min(1.0, max(0.0, (t - leave[i - 1]) / seg))
            return a + (b - a) * u, (b - a) / seg
        if t < leave[i] - 1e-9:                # 在观察点 i 悬停
            return np.asarray(path[i], dtype=float), np.zeros(3)
    return np.asarray(path[n], dtype=float), np.zeros(3)


def fly_legs(wps: Sequence[Waypoint], params: Params, legs: int = 4,
             hover_at: Set[int] = frozenset(),
             progress: bool = False) -> FlyStats:
    """真飞前 legs 个航点段，返回实际航程与到点误差。"""
    Quadrotor, GeometricController, model_dir = _load_backend()
    model_path = os.path.join(model_dir, "quadrotor.xml")

    path = [tuple(map(float, w)) for w in wps[:legs + 1]]
    planned = polyline_length(path)
    hovers = {i for i in hover_at if 0 < i < len(path)}
    arrive, leave = build_timeline(path, hovers, params)
    t_end = leave[-1] + TAIL_S
    n_steps = int(round(t_end / params.dt))

    quad = Quadrotor(model_path)
    quad.reset((path[0][0], path[0][1], max(path[0][2], 0.05)))
    ctrl = GeometricController(quad.params, kw=ATT_DAMPING)

    pos_log: List[np.ndarray] = [quad.qpos[:3].copy()]
    err_log: List[float] = []
    v_filt = np.zeros(3)
    alpha = min(1.0, params.dt / params.ref_tau) if params.ref_tau > 0 else 1.0
    t0_wall = time.time()

    for step in range(n_steps):
        t = step * params.dt
        target, vel = _reference(path, arrive, leave, t)
        v_filt += (vel - v_filt) * alpha
        target[2] = max(target[2], 0.05)
        thrusts = ctrl.act(quad, target, v_filt.copy(), np.zeros(3), 0.0)
        quad.step_with_control(thrusts, 1)
        pos_log.append(quad.qpos[:3].copy())
        err_log.append(float(np.linalg.norm(quad.qpos[:3] - target)))
        if progress and step % 2000 == 0:
            print("  [flyer] sim t=%.1f / %.1f s" % (t, t_end))

    arr = np.asarray(pos_log)
    actual = float(np.linalg.norm(np.diff(arr, axis=0), axis=1).sum())
    return FlyStats(
        legs=legs, planned_m=planned, actual_m=actual,
        arrival_error_m=float(np.linalg.norm(
            arr[-1] - np.asarray(path[-1]))),
        max_track_error_m=max(err_log),
        sim_time_s=n_steps * params.dt, wall_time_s=time.time() - t0_wall)
