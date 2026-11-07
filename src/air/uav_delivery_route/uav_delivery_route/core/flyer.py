#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo closed-loop flyer: fly the first sortie of a plan with uav_mujoco_sim.

单行英文摘要 + 中文正文。
计划层的架次是否可信要靠真飞检验。本模块把第一个架次展开成
"中心 -> 站1(悬停20s) -> 站2(悬停20s) -> ... -> 中心" 的分段参考轨迹，
用课程底座 uav_mujoco_sim（同一份 quadrotor.xml + GeometricController）
跟踪飞行，统计**刚体积分出来的真实位置**：每段实际航程、到站误差、
全程耗时——全部是真值，不是拿计划值凑数。
"""
from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

import numpy as np

#: 参考速度前馈一阶低通时间常数（s），与参考实现同物理约束
REF_TAU = 0.5
#: 姿态角速度阻尼（比底座默认 -0.10 强，抑制分段航迹激励的振荡）
ATT_DAMPING = (-0.18, -0.18, -0.126)
#: 起飞后参考轨迹在中心上空的整备悬停（s）
SETTLE_S = 1.5
#: 落回中心后的收尾悬停（s）
TAIL_S = 1.0


def _load_backend():
    """Import the course sim backend; auto-mount sibling path when run from source."""
    try:
        from uav_mujoco_sim.core.quadrotor import GeometricController, Quadrotor
        from uav_mujoco_sim.core.paths import models_dir
        return Quadrotor, GeometricController, models_dir()
    except Exception:
        here = os.path.abspath(__file__)
        air = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.dirname(here)))))
        sibling = os.path.join(air, "uav_mujoco_sim")
        if os.path.isdir(sibling) and sibling not in sys.path:
            sys.path.insert(0, sibling)
        from uav_mujoco_sim.core.quadrotor import GeometricController, Quadrotor
        from uav_mujoco_sim.core.paths import models_dir
        return Quadrotor, GeometricController, models_dir()


def _waypoints(sortie, depot: Sequence[float], cfg) -> List[Tuple[str, Tuple[float, float, float]]]:
    """Sortie -> named 3D waypoints (depot start/end + one per station)."""
    alt = cfg.cruise_alt_m
    pts = [("depot", (depot[0], depot[1], alt))]
    pts += [(s.sid, (s.x, s.y, alt)) for s in sortie.stations]
    pts.append(("depot", (depot[0], depot[1], alt)))
    return pts


def build_segments(sortie, depot: Sequence[float], cfg) -> List[dict]:
    """Expand waypoints into a time-parameterized reference (legs + delivery hovers).

    段类型 leg：参考位置以巡航速度匀速推进、速度前馈 9 m/s；
    段类型 hold：站上投递悬停 20 s（冒烟测试可用小值压缩仿真时长）。
    """
    wps = _waypoints(sortie, depot, cfg)
    segs: List[dict] = []
    t = 0.0
    segs.append(dict(kind="hold", t0=0.0, t1=SETTLE_S, p0=wps[0][1],
                     p1=wps[0][1], fid=wps[0][0], tid=wps[0][0], planned_m=0.0))
    t = SETTLE_S
    for k in range(len(wps) - 1):
        p0, p1 = np.asarray(wps[k][1]), np.asarray(wps[k + 1][1])
        d = float(np.linalg.norm((p1 - p0)[:2]))
        dur = d / cfg.cruise_speed_ms
        segs.append(dict(kind="leg", t0=t, t1=t + dur, p0=p0, p1=p1,
                         fid=wps[k][0], tid=wps[k + 1][0], planned_m=d))
        t += dur
        if k + 1 < len(wps) - 1:  # 中途站投递悬停，终点中心不悬停
            segs.append(dict(kind="hold", t0=t, t1=t + cfg.hover_s, p0=p1,
                             p1=p1, fid=wps[k + 1][0], tid=wps[k + 1][0],
                             planned_m=0.0))
            t += cfg.hover_s
    segs.append(dict(kind="hold", t0=t, t1=t + TAIL_S, p0=wps[-1][1],
                     p1=wps[-1][1], fid="depot", tid="depot", planned_m=0.0))
    return segs


def ref_at(segs: List[dict], t: float) -> Tuple[np.ndarray, np.ndarray]:
    """Reference position/velocity at time t."""
    if t <= segs[0]["t0"]:
        seg = segs[0]
    elif t >= segs[-1]["t1"]:
        seg = segs[-1]
    else:
        seg = next(s for s in segs if s["t0"] <= t < s["t1"])
    if seg["kind"] == "hold":
        return np.asarray(seg["p1"], float), np.zeros(3)
    p0, p1 = np.asarray(seg["p0"], float), np.asarray(seg["p1"], float)
    u = (t - seg["t0"]) / max(seg["t1"] - seg["t0"], 1e-9)
    d = p1 - p0
    return p0 + d * u, d / max(np.linalg.norm(d), 1e-9) * (
        np.linalg.norm(d[:2]) / max(seg["t1"] - seg["t0"], 1e-9))


@dataclass
class LegReport:
    """One leg: planned vs actually flown."""
    seq: int
    from_id: str
    to_id: str
    planned_dist_m: float
    actual_dist_m: float
    arrival_error_m: float

    def to_dict(self) -> dict:
        return dict(seq=self.seq, from_id=self.from_id, to_id=self.to_id,
                    planned_dist_m=round(self.planned_dist_m, 2),
                    actual_dist_m=round(self.actual_dist_m, 2),
                    arrival_error_m=round(self.arrival_error_m, 3))


@dataclass
class FlightReport:
    """Closed-loop stats for the flown sortie."""
    sortie_index: int
    sim_duration_s: float
    wall_time_s: float
    legs: List[LegReport] = field(default_factory=list)
    hover_count: int = 0

    @property
    def planned_total_m(self) -> float:
        return sum(l.planned_dist_m for l in self.legs)

    @property
    def actual_total_m(self) -> float:
        return sum(l.actual_dist_m for l in self.legs)

    @property
    def mean_arrival_error_m(self) -> float:
        return float(np.mean([l.arrival_error_m for l in self.legs])) if self.legs else 0.0

    @property
    def max_arrival_error_m(self) -> float:
        return max((l.arrival_error_m for l in self.legs), default=0.0)

    def to_dict(self) -> dict:
        return dict(sortie_index=self.sortie_index,
                    sim_duration_s=round(self.sim_duration_s, 2),
                    wall_time_s=round(self.wall_time_s, 2),
                    hover_count=self.hover_count,
                    planned_total_dist_m=round(self.planned_total_m, 2),
                    actual_total_dist_m=round(self.actual_total_m, 2),
                    mean_arrival_error_m=round(self.mean_arrival_error_m, 3),
                    max_arrival_error_m=round(self.max_arrival_error_m, 3),
                    legs=[l.to_dict() for l in self.legs])


def fly_sortie(plan, sortie_index: int, cfg, hover_s: float | None = None,
               dt: float = 0.01, substeps: int = 4, record_every: int = 2,
               max_steps: int = 200000, progress: bool = False) -> FlightReport:
    """Fly one sortie of the plan in MuJoCo and measure what really happened.

    ``hover_s`` 可覆盖配置里的投递悬停时长（冒烟测试用它压缩仿真）。
    """
    Quadrotor, GeometricController, model_dir = _load_backend()
    if hover_s is not None:
        cfg = type(cfg)(**{**cfg.__dict__, "hover_s": hover_s})
    sortie = plan.sorties[sortie_index]
    segs = build_segments(sortie, plan.depot, cfg)
    t_end = segs[-1]["t1"]

    quad = Quadrotor(os.path.join(model_dir, "quadrotor.xml"))
    quad.reset((plan.depot[0], plan.depot[1], 0.05))
    ctrl = GeometricController(quad.params, kw=ATT_DAMPING)

    n_steps = min(max_steps, int(round(t_end / dt)))
    alpha = min(1.0, dt / REF_TAU)
    v_filt = np.zeros(3)

    leg_acc = 0.0
    leg_reports: List[LegReport] = []
    pending_leg = None          # (seg, acc) 刚飞完、等待定误差的 leg
    seg_idx = 0
    prev_pos = quad.qpos[:3].copy()
    wall0 = time.time()

    for step in range(n_steps):
        t = step * dt
        # 段切换：leg 结束先挂起，误差取"到站悬停结束"时刻（投递完成），
        # 即无人机最终停在站点的精度，而不是巡航参考穿越站点的瞬间
        while seg_idx < len(segs) - 1 and t >= segs[seg_idx]["t1"]:
            seg = segs[seg_idx]
            if seg["kind"] == "leg":
                pending_leg = (seg, leg_acc)
                leg_acc = 0.0
            elif seg["kind"] == "hold" and pending_leg is not None:
                pseg, acc = pending_leg
                err = float(np.linalg.norm(prev_pos - np.asarray(seg["p1"])))
                leg_reports.append(LegReport(len(leg_reports), pseg["fid"],
                                             pseg["tid"], pseg["planned_m"],
                                             acc, err))
                pending_leg = None
            seg_idx += 1
        seg = segs[seg_idx]

        p_ref, v_ref = ref_at(segs, t)
        if alpha < 1.0:
            v_filt += (v_ref - v_filt) * alpha
            v_use = v_filt
        else:
            v_use = v_ref
        target = np.asarray(p_ref, float).copy()
        target[2] = max(target[2], 0.05)
        thrusts = ctrl.act(quad, target, v_use, np.zeros(3), 0.0)
        quad.step_with_control(thrusts, substeps)

        pos = quad.qpos[:3]
        if seg["kind"] == "leg":
            leg_acc += float(np.linalg.norm(pos[:2] - prev_pos[:2]))
        prev_pos = pos.copy()

        if progress and step % 5000 == 0:
            print("  [flyer] t=%.1f / %.1f s" % (t, t_end))

    # 收尾段：最后一程 leg 挂尾段悬停结束后结算
    seg = segs[seg_idx]
    if seg["kind"] == "hold" and pending_leg is not None:
        pseg, acc = pending_leg
        err = float(np.linalg.norm(prev_pos - np.asarray(seg["p1"])))
        leg_reports.append(LegReport(len(leg_reports), pseg["fid"], pseg["tid"],
                                     pseg["planned_m"], acc, err))
        pending_leg = None
    sim_t = n_steps * dt
    rep = FlightReport(sortie_index=sortie_index, sim_duration_s=sim_t,
                       wall_time_s=time.time() - wall0,
                       legs=leg_reports,
                       hover_count=sum(1 for s in segs if s["kind"] == "hold") - 2)
    print("[flyer] sortie %d: %d legs, sim %.0f s (wall %.1f s), "
          "actual %.0f m vs planned %.0f m, mean arrival err %.2f m"
          % (sortie_index, len(leg_reports), sim_t, rep.wall_time_s,
             rep.actual_total_m, rep.planned_total_m, rep.mean_arrival_error_m))
    return rep
