#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 闭环复飞：抽取排队前后的若干架次，让无人机真飞对比。

排队结论是否可信，取决于无人机**真飞**之后，实际离地/接地时刻与间隔
是否与排定的一致。本模块用课程既有的 ``uav_mujoco_sim`` 底座（同一份
``quadrotor.xml`` 刚体模型 + 几何控制器）把每架无人机单独实例化，按
三段航迹的位置/速度前馈飞行，记录**刚体积分出来的真实位置**，再从真实
高度越过接地阈值（0.5 m）的时刻提取实际离地/接地时刻。

实测参考：3 架同时仿真 150 s 墙钟约 25 s（约 6 倍实时），纯 CPU。

写法与参考实现 ``_reference_uav_deconflict`` 的 core/sim.py 一致：
参考速度前馈经一阶低通（τ=0.5 s），姿态阻尼 kw≈-0.18，防止分段航迹的
航向阶跃激励姿态回路振荡。
"""
from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .queue import ARRIVAL, DEPARTURE
from .track import Track

#: 参考速度前馈的一阶低通时间常数（s）
REF_TAU = 0.5
#: 姿态角速度阻尼增益（抑制分段航迹激励出的振荡）
ATT_DAMPING = (-0.18, -0.18, -0.126)
#: 接地/离地判定阈值（m）
GROUND_Z = 0.5


def _load_backend():
    """导入课程仿真底座；源码目录直接运行时自动挂同级包路径。"""
    try:
        from uav_mujoco_sim.core.quadrotor import (  # type: ignore
            GeometricController, Quadrotor)
        from uav_mujoco_sim.core.paths import models_dir  # type: ignore
        return Quadrotor, GeometricController, models_dir()
    except Exception:
        here = Path(__file__).resolve()
        # src/air/uav_vertiport_queue/uav_vertiport_queue/core/sim.py → src/air
        sibling = here.parents[3] / "uav_mujoco_sim"
        if sibling.is_dir() and str(sibling) not in sys.path:
            sys.path.insert(0, str(sibling))
        from uav_mujoco_sim.core.quadrotor import (  # type: ignore
            GeometricController, Quadrotor)
        from uav_mujoco_sim.core.paths import models_dir  # type: ignore
        return Quadrotor, GeometricController, models_dir()


@dataclass
class SimResult:
    """真飞记录。"""

    times: np.ndarray
    pos: Dict[str, np.ndarray]                  # uid -> (K, 3) 真实位置
    tracks: Dict[str, Track]
    final_error: Dict[str, float]               # 终点位置误差（m）
    max_track_error: Dict[str, float]           # 全程最大跟踪误差（m）
    steps: int
    wall_time: float

    def op_events(self, z_thresh: float = GROUND_Z) -> Dict[str, dict]:
        """从真实高度提取每架的实际离地/接地时刻。

        离场：真实高度首次超过阈值 → 实际离地时刻；
        进场：真实高度首次低于阈值（起飞于 100 m 巡航高度，全程只降不升）
        → 实际接地时刻。
        """
        out: Dict[str, dict] = {}
        for uid, track in self.tracks.items():
            arr = self.pos[uid]
            z = arr[:, 2]
            if track.kind == DEPARTURE:
                idx = np.argmax(z > z_thresh) if np.any(z > z_thresh) else -1
            else:
                idx = np.argmax(z < z_thresh) if np.any(z < z_thresh) else -1
            actual = float(self.times[idx]) if idx >= 0 else None
            out[uid] = dict(uid=uid, kind=track.kind,
                            planned=round(track.op_time, 2),
                            actual=(round(actual, 2) if actual is not None
                                    else None),
                            error=(round(actual - track.op_time, 2)
                                   if actual is not None else None))
        return out

    def interval_gaps(self, kind: str, z_thresh: float = GROUND_Z) -> List[float]:
        """同类型运行按实际时刻排序后的相邻间隔（s）。"""
        events = sorted(e["actual"] for e in self.op_events(z_thresh).values()
                        if e["kind"] == kind and e["actual"] is not None)
        return [round(b - a, 2) for a, b in zip(events[:-1], events[1:])]


def run_fleet(tracks: Sequence[Track], dt: float = 0.01, substeps: int = 4,
              record_every: int = 5, tail: float = 2.0,
              max_steps: int = 200000, progress: bool = False,
              ref_tau: float = REF_TAU) -> SimResult:
    """让机队按三段航迹真飞。

    参考速度前馈经一阶低通（``ref_tau``）：分段线性航迹在每个航路点都会
    出现速度方向阶跃，直接当年馈会激励姿态回路（参考实现 metro 场景实测
    跟踪误差发散到 2×10^5 m）。

    Parameters
    ----------
    dt
        控制周期（s），模型 ``timestep`` 0.0025 × ``substeps=4`` → 0.01 s。
    record_every
        每多少个控制步记录一次位置（5 → 20 Hz）。
    tail
        最后一个运行结束后继续仿真的时长（s）。
    """
    Quadrotor, GeometricController, model_dir = _load_backend()
    model_path = os.path.join(model_dir, "quadrotor.xml")

    fleet: Dict[str, Tuple[object, object, Track]] = {}
    for track in tracks:
        quad = Quadrotor(model_path)
        p0 = track.segments[0].p0
        quad.reset((float(p0[0]), float(p0[1]), max(0.05, float(p0[2]))))
        ctrl = GeometricController(quad.params, kw=ATT_DAMPING)
        fleet[track.uid] = (quad, ctrl, track)

    t_end = max(tr.end_time for tr in tracks) + tail
    n_steps = min(max_steps, int(round(t_end / dt)))
    times: List[float] = []
    pos: Dict[str, List[np.ndarray]] = {uid: [] for uid in fleet}
    track_err: Dict[str, List[float]] = {uid: [] for uid in fleet}
    v_filt: Dict[str, np.ndarray] = {uid: np.zeros(3) for uid in fleet}
    alpha = min(1.0, dt / ref_tau) if ref_tau and ref_tau > 0 else 1.0

    t0_wall = time.time()
    for step in range(n_steps):
        t = step * dt
        for uid, (quad, ctrl, track) in fleet.items():
            target = track.hold_position(t)
            vel = track.velocity_at(t)
            if vel is None:
                vel = np.zeros(3)
            if alpha < 1.0:
                v_filt[uid] += (vel - v_filt[uid]) * alpha
                vel = v_filt[uid]
            target = target.copy()
            target[2] = max(target[2], 0.05)
            thrusts = ctrl.act(quad, target, vel, np.zeros(3), 0.0)
            quad.step_with_control(thrusts, substeps)
            if step % record_every == 0 and track.is_active(t):
                err = float(np.linalg.norm(quad.qpos[:3] - track.position_at(t)))
                track_err[uid].append(err)

        if step % record_every == 0:
            times.append(t)
            for uid, (quad, _c, _t) in fleet.items():
                pos[uid].append(quad.qpos[:3].copy())

        if progress and step % 5000 == 0:
            print("    sim t=%.1f / %.1f s" % (t, t_end))

    times.append(n_steps * dt)
    for uid, (quad, _c, _t) in fleet.items():
        pos[uid].append(quad.qpos[:3].copy())

    arr = {uid: np.asarray(v, dtype=float) for uid, v in pos.items()}
    final = {}
    for uid, (_q, _c, track) in fleet.items():
        p1 = track.segments[-1].p1
        final[uid] = float(np.linalg.norm(arr[uid][-1] - p1))
    max_err = {uid: (max(v) if v else 0.0) for uid, v in track_err.items()}

    return SimResult(times=np.asarray(times, dtype=float), pos=arr,
                     tracks={t.uid: t for t in tracks},
                     final_error=final, max_track_error=max_err,
                     steps=n_steps, wall_time=time.time() - t0_wall)
