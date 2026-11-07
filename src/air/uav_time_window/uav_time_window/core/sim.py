#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 闭环真飞：抽 2 架机的任务序列按 4D 航迹执行，统计实际到达时刻。

分配与时间窗检验都发生在"计划空间"里，结论是否可信取决于真飞之后每个
任务点的实际到达时刻还落不落在窗口附近。本模块用课程既有的
``uav_mujoco_sim`` 底座（同一份 ``quadrotor.xml`` 刚体模型 + 几何控制器）
把选中的飞机单独实例化，按 ``RouteTrack`` 的位置/速度前馈飞行，记录
**刚体积分出来的真实位置**，再据"首次水平进入任务点半径"判定实际到达
时刻，统计与计划到达及时间窗的偏差。

实测参考：2 架同时仿真 100 s 墙钟约 15 s（约 7 倍实时），纯 CPU。
"""
from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

from .track import RouteTrack

#: 参考速度前馈的一阶低通时间常数（s）
REF_TAU = 0.5
#: 姿态角速度阻尼增益（比底座默认 −0.10 略强，抑制分段航迹激励出的振荡）
ATT_DAMPING = (-0.18, -0.18, -0.126)
#: 实际到达判定半径（m，水平方向）与高度容差（m）
ARRIVE_RADIUS = 5.0
ARRIVE_ALT_TOL = 8.0


def _load_backend():
    """导入课程仿真底座；源码目录直接运行时自动挂同级包路径。"""
    try:
        from uav_mujoco_sim.core.quadrotor import (  # type: ignore
            GeometricController, Quadrotor)
        from uav_mujoco_sim.core.paths import models_dir  # type: ignore
        return Quadrotor, GeometricController, models_dir()
    except Exception:
        here = Path(__file__).resolve()
        # src/air/uav_time_window/uav_time_window/core/sim.py → src/air
        sibling = here.parents[3] / "uav_mujoco_sim"
        if sibling.is_dir() and str(sibling) not in sys.path:
            sys.path.insert(0, str(sibling))
        from uav_mujoco_sim.core.quadrotor import (  # type: ignore
            GeometricController, Quadrotor)
        from uav_mujoco_sim.core.paths import models_dir  # type: ignore
        return Quadrotor, GeometricController, models_dir()


@dataclass
class ArrivalRecord:
    """单个任务点的实际到达统计。"""

    uid: str
    tid: str
    planned: float          # 计划到达时刻
    actual: Optional[float]  # 实际到达时刻（未检出为 None）
    window: tuple           # (earliest, latest)

    @property
    def deviation(self) -> Optional[float]:
        """实际 − 计划到达偏差（s）。"""
        return None if self.actual is None else self.actual - self.planned

    @property
    def lateness(self) -> Optional[float]:
        """相对窗口的实际晚到（s，负值=早到）。"""
        if self.actual is None:
            return None
        serve = self.actual
        if serve < self.window[0]:
            return serve - self.window[0]   # 早到
        return max(0.0, serve - self.window[1])

    def as_dict(self) -> dict:
        return dict(uid=self.uid, tid=self.tid,
                    planned=round(self.planned, 2),
                    actual=round(self.actual, 2) if self.actual is not None else None,
                    window=[round(self.window[0], 2), round(self.window[1], 2)],
                    deviation=round(self.deviation, 2)
                    if self.deviation is not None else None,
                    status=("missed" if self.actual is None else
                            ("early" if self.actual < self.window[0] - 1e-9
                             else ("late" if self.actual > self.window[1] + 1e-9
                                   else "on-time"))))


@dataclass
class SimResult:
    """真飞记录。"""

    times: np.ndarray
    pos: Dict[str, np.ndarray]                 # uid -> (K, 3) 真实位置
    tracks: Dict[str, RouteTrack]
    arrivals: List[ArrivalRecord] = field(default_factory=list)
    max_track_error: Dict[str, float] = field(default_factory=dict)
    steps: int = 0
    wall_time: float = 0.0

    def arrivals_as_dict(self) -> List[dict]:
        return [a.as_dict() for a in self.arrivals]

    def summary(self) -> dict:
        devs = [abs(a.deviation) for a in self.arrivals
                if a.deviation is not None]
        missed = [a.tid for a in self.arrivals if a.actual is None]
        return dict(n_arrivals=len(self.arrivals),
                    deviation_mean=round(float(np.mean(devs)), 2) if devs else None,
                    deviation_max=round(float(np.max(devs)), 2) if devs else None,
                    missed=missed)


def run_fleet(tracks: Sequence[RouteTrack], dt: float = 0.01, substeps: int = 4,
              record_every: int = 5, tail: float = 2.0,
              max_steps: int = 200000, progress: bool = False,
              ref_tau: float = REF_TAU) -> SimResult:
    """让选中的飞机按各自 4D 航迹真飞。

    参考速度前馈经一阶低通（``ref_tau``）。分段线性航迹在每个航路点都会
    出现速度方向阶跃，直接当年馈会激励姿态回路（参考实现实测整机翻转
    坠落），真实飞控本来就有轨迹平滑器，这里按同一物理约束补上。
    """
    Quadrotor, GeometricController, model_dir = _load_backend()
    model_path = os.path.join(model_dir, "quadrotor.xml")

    fleet = {}
    for tr in tracks:
        quad = Quadrotor(model_path)
        x0, y0 = tr.uav.pos
        quad.reset((x0, y0, 0.05))
        ctrl = GeometricController(quad.params, kw=ATT_DAMPING)
        fleet[tr.uid] = (quad, ctrl, tr)

    t_end = max((t.end_time for t in tracks), default=0.0) + tail
    n_steps = min(max_steps, int(round(t_end / dt)))
    times: List[float] = []
    pos: Dict[str, List[np.ndarray]] = {uid: [] for uid in fleet}
    track_err: Dict[str, List[float]] = {uid: [] for uid in fleet}
    v_filt: Dict[str, np.ndarray] = {uid: np.zeros(3) for uid in fleet}
    alpha = min(1.0, dt / ref_tau) if ref_tau and ref_tau > 0 else 1.0

    t0_wall = time.time()
    for step in range(n_steps):
        t = step * dt
        for uid, (quad, ctrl, tr) in fleet.items():
            target = tr.hold_position(t)
            vel = tr.velocity_at(t)
            if vel is None:
                vel = np.zeros(3)
            if alpha < 1.0:
                v_filt[uid] += (vel - v_filt[uid]) * alpha
                vel = v_filt[uid]
            target = target.copy()
            target[2] = max(target[2], 0.05)
            thrusts = ctrl.act(quad, target, vel, np.zeros(3), 0.0)
            quad.step_with_control(thrusts, substeps)
            if step % record_every == 0 and tr.start_time - 1e-9 <= t <= tr.end_time + 1e-9:
                err = float(np.linalg.norm(quad.qpos[:3] - tr.position_at(t)))
                track_err[uid].append(err)

        if step % record_every == 0:
            times.append(t)
            for uid, (quad, _c, _t) in fleet.items():
                pos[uid].append(quad.qpos[:3].copy())

        if progress and step % 5000 == 0:
            print("  sim t=%.1f / %.1f s" % (t, t_end))

    times.append(n_steps * dt)
    for uid, (quad, _c, _t) in fleet.items():
        pos[uid].append(quad.qpos[:3].copy())

    arr = {uid: np.asarray(v, dtype=float) for uid, v in pos.items()}
    max_err = {uid: (max(v) if v else 0.0) for uid, v in track_err.items()}

    result = SimResult(times=np.asarray(times, dtype=float), pos=arr,
                       tracks={t.uid: t for t in tracks},
                       max_track_error=max_err, steps=n_steps,
                       wall_time=time.time() - t0_wall)
    result.arrivals = detect_arrivals(result)
    return result


def detect_arrivals(result: SimResult,
                    radius: float = ARRIVE_RADIUS,
                    alt_tol: float = ARRIVE_ALT_TOL) -> List[ArrivalRecord]:
    """按"首次水平进入任务点半径且高度接近巡航高度"判定实际到达时刻。"""
    out: List[ArrivalRecord] = []
    for uid in sorted(result.pos):
        tr = result.tracks[uid]
        p = result.pos[uid]
        for stop in tr.schedule.stops:
            task = tr.tasks[stop.tid]
            dh = np.linalg.norm(p[:, :2] - np.array(task.pos), axis=1)
            ok = (dh < radius) & (np.abs(p[:, 2] - tr.alt) < alt_tol)
            idx = int(np.argmax(ok)) if np.any(ok) else -1
            actual = float(result.times[idx]) if idx >= 0 else None
            out.append(ArrivalRecord(uid=uid, tid=stop.tid,
                                     planned=tr.planned_arrival[stop.tid],
                                     actual=actual,
                                     window=(task.earliest, task.latest)))
    return out
