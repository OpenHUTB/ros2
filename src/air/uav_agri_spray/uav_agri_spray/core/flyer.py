#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo closed-loop flight of the first N scan lines, real deposition stats.

闭环验证：抽推荐参数下的前 N 条扫描线交给课程底座 uav_mujoco_sim
（Quadrotor + GeometricController）真飞，参考速度前馈经一阶低通
（τ=0.5 s）、姿态阻尼 kw≈-0.18 抑制分段航迹激励的振荡；用刚体积分出的
真实位置重算实际沉积栅格与覆盖率，与计划值对比证明闭环。
"""
from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Tuple

import numpy as np

from .checker import (DepositMap, DepositMetrics, build_grid,
                      accumulate_deposit_points, evaluate_deposition)
from .geometry import segment_point_distance
from .planner import SprayConfig, SprayPath

TAG = "[flyer]"

#: 参考速度前馈一阶低通时间常数（s），与参考实现一致
REF_TAU = 0.5
#: 姿态角速度阻尼（比底座默认略强，抑制航向/转角阶跃激励）
ATT_DAMPING = (-0.18, -0.18, -0.126)


@dataclass(frozen=True)
class FlightConfig:
    """Sim loop knobs (kept small so a smoke flight stays under ~10 s wall)."""

    dt: float = 0.01
    substeps: int = 4
    record_every: int = 5
    climb_rate: float = 1.5      # 起飞爬升率 m/s
    tail: float = 3.0            # 终点悬停 s（让低通滞后收敛）
    ref_tau: float = REF_TAU


@dataclass
class FlightResult:
    """Closed-loop outcome: real trajectory + real deposition metrics."""

    times: np.ndarray
    positions: np.ndarray            # (K, 3) 真实位置
    flown_lines: int
    flown_length_m: float
    t_climb: float
    max_track_error_m: float
    final_error_m: float
    steps: int
    wall_time_s: float
    actual: DepositMetrics
    planned: DepositMetrics
    actual_map: dict = None      # 真实沉积栅格（JSON 序列化用）


def _load_backend():
    """Import uav_mujoco_sim; auto-mount sibling path for source runs."""
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


def trim_to_lines(path: SprayPath, n_lines: int) -> np.ndarray:
    """Cut the mission polyline at the end of line n_lines-1."""
    keep = path.points[:0]
    last = -1
    for i, lid in enumerate(path.seg_line):
        if int(lid) <= n_lines - 1:
            last = i
    if last < 0:
        return path.points[:2]
    return path.points[:last + 2]


def make_sampler(points: np.ndarray, speed: float, t_climb: float,
                 climb_rate: float) -> Tuple[Callable, float]:
    """Constant-speed trajectory along a polyline; hover+climb before t_climb.

    返回 (sample(t) -> (pos(3,), vel(3,)), 总时长 t_climb + 航程/speed)。
    """
    d = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(d)])
    total_len = float(cum[-1])
    t_end = t_climb + total_len / speed

    def sample(t: float) -> Tuple[np.ndarray, np.ndarray]:
        t = float(np.clip(t, 0.0, t_end))
        p0 = points[0]
        if t < t_climb:
            z = min(0.05 + climb_rate * t, 0.05 + climb_rate * t_climb)
            return np.array([p0[0], p0[1], z]), np.zeros(3)
        s = min((t - t_climb) * speed, total_len)
        i = int(np.searchsorted(cum, s, side="right") - 1)
        i = max(0, min(i, len(points) - 2))
        seg = points[i + 1] - points[i]
        L = max(float(np.linalg.norm(seg)), 1e-9)
        u = seg / L
        frac = s - cum[i]
        p = points[i] + u * frac
        v = np.zeros(3) if s >= total_len - 1e-6 else np.array([u[0], u[1], 0.0]) * speed
        return np.array([p[0], p[1], 0.0]), v

    return sample, t_end


def track_error(points: np.ndarray, xy: np.ndarray) -> float:
    """Distance from a position to the planned polyline."""
    best = np.inf
    for i in range(len(points) - 1):
        dd = segment_point_distance(points[i], points[i + 1],
                                    xy.reshape(1, 2))[0]
        best = min(best, float(dd))
    return best


def fly_first_lines(path: SprayPath, n_lines: int, spray_cfg: SprayConfig,
                    outline: np.ndarray, flight_cfg: FlightConfig = None,
                    progress: bool = False) -> FlightResult:
    """Fly the first n_lines scan lines in MuJoCo and re-assess deposition."""
    Quadrotor, GeometricController, model_dir = _load_backend()
    flight_cfg = flight_cfg or FlightConfig()
    sub = trim_to_lines(path, n_lines)
    flown_len = float(np.linalg.norm(np.diff(sub, axis=0), axis=1).sum())
    t_climb = spray_cfg.alt / flight_cfg.climb_rate
    sample, t_end = make_sampler(sub, spray_cfg.speed, t_climb,
                                 flight_cfg.climb_rate)
    t_total = t_end + flight_cfg.tail

    quad = Quadrotor(os.path.join(model_dir, "quadrotor.xml"))
    quad.reset((float(sub[0][0]), float(sub[0][1]), 0.05))
    ctrl = GeometricController(quad.params, kw=ATT_DAMPING)

    n_steps = int(round(t_total / flight_cfg.dt))
    alpha = min(1.0, flight_cfg.dt / flight_cfg.ref_tau)
    v_filt = np.zeros(3)
    times, poss = [], []
    errs = []
    t0 = time.time()
    for step in range(n_steps):
        t = step * flight_cfg.dt
        p_ref, v_ref = sample(t)
        p_ref = p_ref.copy()
        if t >= t_climb:
            p_ref[2] = spray_cfg.alt
        p_ref[2] = max(p_ref[2], 0.05)
        if alpha < 1.0:
            v_filt += (v_ref - v_filt) * alpha
            v_use = v_filt.copy()
        else:
            v_use = v_ref
        thrusts = ctrl.act(quad, p_ref, v_use, np.zeros(3), 0.0)
        quad.step_with_control(thrusts, flight_cfg.substeps)
        if step % flight_cfg.record_every == 0 and t >= t_climb:
            errs.append(track_error(sub, quad.qpos[:2]))
        if step % flight_cfg.record_every == 0:
            times.append(t)
            poss.append(quad.qpos[:3].copy())
        if progress and step % 5000 == 0:
            print("  %s sim t=%.1f / %.1f s" % (TAG, t, t_total))
    times.append(n_steps * flight_cfg.dt)
    poss.append(quad.qpos[:3].copy())

    times_arr = np.asarray(times)
    pos_arr = np.asarray(poss)
    final_err = float(np.linalg.norm(pos_arr[-1, :2] - sub[-1]))

    # 真实沉积：只取运动中的采样点（排除终点悬停抖动），按 0.3 m 重采样
    xy = pos_arr[:, :2]
    dt_rec = np.diff(times_arr)
    speed_rec = np.linalg.norm(np.diff(xy, axis=0), axis=1) / np.maximum(
        dt_rec, 1e-9)
    moving = np.concatenate([[True], speed_rec > 0.5])
    xy = xy[moving]
    if len(xy) < 2:
        xy = pos_arr[:, :2]
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    keep = np.concatenate([[True], seg > 1e-3])
    xy = xy[keep]
    cum = np.concatenate([[0.0], np.cumsum(
        np.linalg.norm(np.diff(xy, axis=0), axis=1))])
    n_rs = max(2, int(cum[-1] / 0.3))
    s_rs = np.linspace(0.0, cum[-1], n_rs)
    x_rs = np.interp(s_rs, cum, xy[:, 0])
    y_rs = np.interp(s_rs, cum, xy[:, 1])

    grid: DepositMap = build_grid(outline, 1.0)
    dep = accumulate_deposit_points(grid, np.stack([x_rs, y_rs], axis=1),
                                    spray_cfg.sigma)
    grid.deposit = dep
    actual = evaluate_deposition(dep, grid.inside, 1.0)
    actual_map = {
        "x0": grid.x0, "y0": grid.y0, "resolution": grid.resolution,
        "nx": int(grid.deposit.shape[1]), "ny": int(grid.deposit.shape[0]),
        "deposit": np.round(grid.deposit, 3).tolist(),
        "route": np.round(xy, 2).tolist(),
    }

    # 计划值：同样前 n_lines 行的喷洒段（直行+绕行弧）在同一栅格上评估
    from .checker import accumulate_deposit
    plan_lines = [ln for ln in path.lines if ln.line_id < n_lines]
    plan_segs = [seg for ln in plan_lines
                 for seg in (list(ln.straight) + list(ln.arcs))]
    dep_p = accumulate_deposit(grid, plan_segs, spray_cfg.sigma)
    planned = evaluate_deposition(dep_p, grid.inside, 1.0)

    return FlightResult(
        times=times_arr, positions=pos_arr, flown_lines=n_lines,
        flown_length_m=round(flown_len, 1), t_climb=t_climb,
        max_track_error_m=round(max(errs) if errs else 0.0, 3),
        final_error_m=round(final_err, 3),
        steps=n_steps, wall_time_s=round(time.time() - t0, 1),
        actual=actual, planned=planned, actual_map=actual_map)
