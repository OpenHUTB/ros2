#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 闭环复飞：让每架无人机按消解后的 4D 航迹真飞一遍。

冲突检测与解脱都发生在"计划空间"里，结论是否可信取决于真飞之后间隔还
在不在。本模块用课程既有的 ``uav_mujoco_sim`` 底座（同一份 ``quadrotor.xml``
刚体模型 + 几何控制器）把每架飞机单独实例化，按航迹的位置/速度前馈飞行，
记录**刚体积分出来的真实位置**，再据此统计实际最小间隔。

实测参考：4 架同时仿真 20 s 墙钟约 2.8 s（约 7 倍实时），纯 CPU。
"""
from __future__ import annotations

import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .conflict import Separation
from .plan import Plan

#: 参考速度前馈的一阶低通时间常数（s）
REF_TAU = 0.5
#: 姿态角速度阻尼增益（比底座默认 −0.10 略强，抑制分段航迹激励出的振荡）
ATT_DAMPING = (-0.18, -0.18, -0.126)


def _load_backend():
    """导入课程仿真底座；源码目录直接运行时自动挂同级包路径。"""
    try:
        from uav_mujoco_sim.core.quadrotor import (  # type: ignore
            GeometricController, Quadrotor)
        from uav_mujoco_sim.core.paths import models_dir  # type: ignore
        return Quadrotor, GeometricController, models_dir()
    except Exception:
        here = Path(__file__).resolve()
        # src/air/uav_deconflict/uav_deconflict/core/sim.py → src/air
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
    pos: Dict[str, np.ndarray]                 # uid -> (K, 3) 真实位置
    plans: Dict[str, Plan]
    final_error: Dict[str, float]              # 终点位置误差（m）
    max_track_error: Dict[str, float]          # 全程最大跟踪误差（m）
    steps: int
    wall_time: float
    target: Dict[str, np.ndarray] = field(default_factory=dict)

    def min_separation(self, sep: Optional[Separation] = None) -> dict:
        """按圆柱间隔模型统计真飞中的最小间隔。

        只在"垂直间隔已经小于阈值"的时刻统计水平间隔——这正对应
        "水平与垂直同时侵入"的冲突定义，避免把一架已落地、另一架从
        正上方飞过误判成危险接近。
        """
        sep = sep or Separation()
        best_h = math.inf
        best_pair = ("", "")
        best_t = 0.0
        min_v_any = math.inf
        for i, a in enumerate(sorted(self.pos)):
            for b in sorted(self.pos)[i + 1:]:
                pa, pb = self.pos[a], self.pos[b]
                d = pa - pb
                h = np.linalg.norm(d[:, :2], axis=1)
                v = np.abs(d[:, 2])
                min_v_any = min(min_v_any, float(v.min()))
                mask = v < sep.vertical
                if not np.any(mask):
                    continue
                idx = int(np.argmin(np.where(mask, h, np.inf)))
                if h[idx] < best_h:
                    best_h = float(h[idx])
                    best_pair = (a, b)
                    best_t = float(self.times[idx])
        return dict(min_h=best_h, min_v=min_v_any, pair=best_pair,
                    t=best_t, sep=sep)

    def violations(self, sep: Separation) -> List[Tuple[float, str, str, float, float]]:
        """逐时刻列出间隔侵入（真飞）。"""
        out = []
        names = sorted(self.pos)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                d = self.pos[a] - self.pos[b]
                h = np.linalg.norm(d[:, :2], axis=1)
                v = np.abs(d[:, 2])
                bad = np.where((h < sep.horizontal) & (v < sep.vertical))[0]
                for k in bad:
                    out.append((float(self.times[k]), a, b,
                                float(h[k]), float(v[k])))
        return out


def run_fleet(plans: Sequence[Plan], dt: float = 0.01, substeps: int = 4,
              record_every: int = 5, tail: float = 2.0,
              max_steps: int = 200000, progress: bool = False,
              ref_tau: float = REF_TAU) -> SimResult:
    """让机队按航迹真飞。

    参考速度前馈经一阶低通（``ref_tau``）。分段线性航迹在每个航路点都会
    出现速度方向阶跃，直接当年馈会激励姿态回路：metro 场景实测 UAV-6 在
    第二个航路点被激成 10 Hz 增幅振荡、整机翻转坠落（跟踪误差发散到
    2×10^5 m）。真实飞控本来就有轨迹平滑器，这里按同一物理约束补上。

    Parameters
    ----------
    dt
        控制周期（s），模型 ``timestep`` 为 0.0025，``substeps=4`` 对应 0.01 s。
    record_every
        每多少个控制步记录一次位置（5 → 20 Hz）。
    tail
        最后一架落地后继续悬停/停放的时长（s）。
    ref_tau
        参考速度一阶低通时间常数（s）；≤0 表示不平滑。
    """
    Quadrotor, GeometricController, model_dir = _load_backend()
    model_path = os.path.join(model_dir, "quadrotor.xml")

    fleet: Dict[str, Tuple[object, object, Plan]] = {}
    for plan in plans:
        quad = Quadrotor(model_path)
        x0, y0 = plan.path[0]
        quad.reset((x0, y0, 0.05))
        ctrl = GeometricController(quad.params, kw=ATT_DAMPING)
        fleet[plan.uid] = (quad, ctrl, plan)

    t_end = max(p.end_time for p in plans) + tail
    n_steps = min(max_steps, int(round(t_end / dt)))
    times: List[float] = []
    pos: Dict[str, List[np.ndarray]] = {uid: [] for uid in fleet}
    track_err: Dict[str, List[float]] = {uid: [] for uid in fleet}
    v_filt: Dict[str, np.ndarray] = {uid: np.zeros(3) for uid in fleet}
    alpha = min(1.0, dt / ref_tau) if ref_tau and ref_tau > 0 else 1.0

    t0_wall = time.time()
    for step in range(n_steps):
        t = step * dt
        for uid, (quad, ctrl, plan) in fleet.items():
            target = plan.hold_position(t)
            vel = plan.velocity_at(t)
            if vel is None:
                vel = np.zeros(3)
            if alpha < 1.0:
                v_filt[uid] += (vel - v_filt[uid]) * alpha
                vel = v_filt[uid]
            target = target.copy()
            target[2] = max(target[2], 0.05)
            thrusts = ctrl.act(quad, target, vel, np.zeros(3), 0.0)
            quad.step_with_control(thrusts, substeps)
            if step % record_every == 0 and plan.is_active(t):
                err = float(np.linalg.norm(quad.qpos[:3] - plan.position_at(t)))
                track_err[uid].append(err)

        if step % record_every == 0:
            times.append(t)
            for uid, (quad, _c, _p) in fleet.items():
                pos[uid].append(quad.qpos[:3].copy())

        if progress and step % 5000 == 0:
            print("  sim t=%.1f / %.1f s" % (t, t_end))

    # 收尾：再记录一帧真实终点
    times.append(n_steps * dt)
    for uid, (quad, _c, _p) in fleet.items():
        pos[uid].append(quad.qpos[:3].copy())

    arr = {uid: np.asarray(v, dtype=float) for uid, v in pos.items()}
    final = {}
    for uid, (_q, _c, plan) in fleet.items():
        xe, ye = plan.path[-1]
        final[uid] = float(np.linalg.norm(arr[uid][-1, :2] - np.array([xe, ye])))
    max_err = {uid: (max(v) if v else 0.0) for uid, v in track_err.items()}

    return SimResult(times=np.asarray(times, dtype=float), pos=arr,
                     plans={p.uid: p for p in plans},
                     final_error=final, max_track_error=max_err,
                     steps=n_steps, wall_time=time.time() - t0_wall)
