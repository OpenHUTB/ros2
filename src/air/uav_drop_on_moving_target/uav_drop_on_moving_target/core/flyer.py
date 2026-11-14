#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 闭环：沿标称航迹飞到释放点，按真实状态判定松手，复核落点。

一行摘要：参考航迹来自 track 模块，松手条件来自 release.evaluate_release，
落点由真实释放状态外推——三者与节点用的是同一套函数，闭环里不存在
"另一套判据"。
"""
from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from .ballistics import impact_point
from .release import DropCfg, evaluate_release, solve_release
from .track import PolylineTrack
from .vehicle import Vehicle

CLIMB_SPEED = 2.5          # m/s
GROUND_Z = 0.05
#: 参考点严格按时间推进；只有无人机落后超过该距离时才把它拽近，避免
#: 位置误差项顶到倾角上限（把参考点改成"机体前方 3 m"会变成追自己的尾巴，
#: 参考速度与机体速度同步，永远差 3 m 追不上）
MAX_LEAD_M = 20.0
REF_TAU = 0.5              # 参考速度一阶低通（s）
ATT_DAMPING = (-0.18, -0.18, -0.126)
YAW_GAIN = 1.0


def _load_backend():
    """导入课程仿真底座；源码目录直接运行时自动挂同级包路径。"""
    try:
        from uav_mujoco_sim.core.quadrotor import (  # type: ignore
            GeometricController, Quadrotor)
        from uav_mujoco_sim.core.paths import models_dir  # type: ignore
        return Quadrotor, GeometricController, models_dir()
    except Exception:                                     # noqa: BLE001
        here = Path(__file__).resolve()
        sibling = here.parents[3] / "uav_mujoco_sim"
        if sibling.is_dir() and str(sibling) not in sys.path:
            sys.path.insert(0, str(sibling))
        from uav_mujoco_sim.core.quadrotor import (  # type: ignore
            GeometricController, Quadrotor)
        from uav_mujoco_sim.core.paths import models_dir  # type: ignore
        return Quadrotor, GeometricController, models_dir()


class _PathRef:
    """沿标称航迹推进的参考点，带 3 m 拴绳。"""

    def __init__(self, track: PolylineTrack, speed: float):
        self.track = track
        self.speed = float(speed)
        self._segs = track.segs
        self._len = [s[2] for s in self._segs]

    def nominal_at(self, s: float) -> Tuple[np.ndarray, np.ndarray]:
        s = max(0.0, float(s))
        for (a, b, n, u), _l in zip(self._segs, self._len):
            if s <= n:
                return a + u * s, u * self.speed
            s -= n
        a, b, n, u = self._segs[-1]
        return b.copy(), np.zeros(2)

    def ref(self, elapsed: float, pos: np.ndarray,
            alt_m: float) -> Tuple[np.ndarray, np.ndarray]:
        # 与解算器共用 track.pos_vel：起步加速段也算在标称航迹里
        nominal, tangent = self.track.pos_vel(max(0.0, elapsed))
        d = nominal - pos[:2]
        dist = float(np.linalg.norm(d))
        if dist > MAX_LEAD_M:
            nominal = pos[:2] + d / dist * MAX_LEAD_M
        p_ref = np.array([nominal[0], nominal[1], alt_m])
        v_ref = np.array([tangent[0], tangent[1], 0.0])
        return p_ref, v_ref


@dataclass
class DropRecord:
    """一次投放的真实记录。"""

    t_release: float
    release_pos: np.ndarray
    release_vel: np.ndarray
    t_fall: float
    impact_xy: np.ndarray
    vehicle_at_impact: np.ndarray
    vehicle_at_release: np.ndarray
    error_m: float
    reason: str = ""

    def as_dict(self) -> dict:
        return dict(
            t_release_s=round(float(self.t_release), 2),
            release_pos_m=[round(float(v), 2) for v in self.release_pos],
            release_vel_mps=[round(float(v), 2) for v in self.release_vel],
            t_fall_s=round(float(self.t_fall), 3),
            impact_m=[round(float(v), 2) for v in self.impact_xy],
            vehicle_at_release_m=[round(float(v), 2) for v in self.vehicle_at_release],
            vehicle_at_impact_m=[round(float(v), 2) for v in self.vehicle_at_impact],
            error_m=round(float(self.error_m), 2),
            reason=self.reason)


@dataclass
class FlyResult:
    """闭环真飞结果。"""

    times: np.ndarray
    pos: np.ndarray                    # (K, 3) 真实位置
    track_xy: np.ndarray               # (M, 2) 标称航迹折线
    vehicle_track: np.ndarray          # (M, 2) 车辆走过的点（采样）
    releases: List[DropRecord]
    max_track_error_m: float           # 爬升结束 4 s 后的航迹跟踪误差
    max_error_release_m: float         # 释放窗口 ±1 s 内的跟踪误差
    final_error_m: float
    sim_s: float
    wall_s: float

    def as_dict(self) -> dict:
        errs = [r.error_m for r in self.releases]
        return dict(
            simulated=True,
            releases=[r.as_dict() for r in self.releases],
            impact_error_mean_m=round(float(np.mean(errs)), 2) if errs else None,
            impact_error_max_m=round(float(np.max(errs)), 2) if errs else None,
            max_track_error_m=round(self.max_track_error_m, 2),
            max_error_near_release_m=round(self.max_error_release_m, 2),
            final_track_error_m=round(self.final_error_m, 2),
            sim_duration_s=round(self.sim_s, 1),
            wall_time_s=round(self.wall_s, 1))


def fly_case(track: PolylineTrack, vehicle: Vehicle, cfg: DropCfg,
             dt: float = 0.01, record_every: int = 5,
             progress: bool = False) -> FlyResult:
    """飞完一条航迹并完成一次投放（MuJoCo 真实动力学）。"""
    Quadrotor, GeometricController, models = _load_backend()
    xml = str(Path(models) / "quadrotor.xml")

    solution = solve_release(track, vehicle, cfg)
    if not solution.feasible:
        raise RuntimeError("infeasible case: %s" % solution.advice)
    t_release = float(solution.t_release)

    quad = Quadrotor(xml)
    ctrl = GeometricController(quad.params, kw=ATT_DAMPING)
    start = np.asarray(track.points[0], dtype=float)
    quad.reset((start[0], start[1], GROUND_Z))
    climb_s = cfg.climb_s
    t_end = t_release + 6.0        # 投放后再飞 6 s 收尾，不飞到航迹末端
    n_steps = int(t_end / dt)

    pref = _PathRef(track, cfg.speed)
    log_t, log_p = [], []
    releases: List[DropRecord] = []
    track_err = []          # (t, 误差) 用于统计
    released = False
    v_filt = np.zeros(3)          # 参考速度一阶低通，抑制分段切换处的阶跃
    alpha = min(1.0, dt / REF_TAU)
    t_wall0 = time.time()

    for step in range(n_steps):
        t = step * dt
        pos = quad.qpos[:3].copy()
        vel = quad.qvel[:3].copy()

        if t < climb_s:
            frac = min(t / max(climb_s, 1e-9), 1.0)
            p_ref = np.array([start[0], start[1],
                              GROUND_Z + (cfg.alt_m - GROUND_Z) * frac])
            v_ref = np.array([0.0, 0.0, CLIMB_SPEED if frac < 1.0 else 0.0])
            yaw_d = 0.0
        else:
            p_ref, v_ref = pref.ref(t - climb_s, pos, cfg.alt_m)
            yaw_d = 0.0
            track_err.append((t, float(np.linalg.norm(pos[:2] - p_ref[:2]))))

            # 松手判定：到点之后再看真实状态能不能投中（同一套判据）
            if not released and t >= t_release:
                verdict = evaluate_release(pos, vel, t, vehicle, cfg)
                if verdict["ok"]:
                    released = True
                    t_fall = float(verdict["t_fall"])
                    imp = np.asarray(verdict["impact_xy"], dtype=float)
                    tgt = np.asarray(verdict["target_xy"], dtype=float)
                    releases.append(DropRecord(
                        t_release=t, release_pos=pos.copy(), release_vel=vel.copy(),
                        t_fall=t_fall, impact_xy=imp, vehicle_at_impact=tgt,
                        vehicle_at_release=vehicle.pos_at(t),
                        error_m=float(np.linalg.norm(imp - tgt))))

        v_filt = v_filt + alpha * (v_ref - v_filt)
        thrusts = ctrl.act(quad, p_ref, v_filt, np.zeros(3), yaw_d)
        quad.step_with_control(thrusts, 1)

        if step % record_every == 0:
            log_t.append(t)
            log_p.append(pos)

    wall = time.time() - t_wall0
    times = np.asarray(log_t)
    pos_log = np.asarray(log_p)

    # 爬升→巡航有一记速度阶跃（0 → 10 m/s），机体要几秒才追上参考点；
    # 这段一次性追赶单独计入 max_track_error_m，投放窗口内另算
    settled = [(t, e) for t, e in track_err if t > climb_s + 4.0]
    near = [(t, e) for t, e in track_err
            if abs(t - t_release) <= 1.0]
    final = [e for t, e in track_err if t > t_end - 0.8]
    vehicle_pts = np.asarray([vehicle.pos_at(t) for t in np.linspace(
        0.0, min(vehicle.duration_s, t_end), 80)])

    return FlyResult(
        times=times, pos=pos_log,
        track_xy=np.asarray(track.points, dtype=float),
        vehicle_track=vehicle_pts, releases=releases,
        max_track_error_m=max((e for _t, e in settled), default=float("nan")),
        max_error_release_m=max((e for _t, e in near), default=float("nan")),
        final_error_m=float(np.mean(final)) if final else float("nan"),
        sim_s=float(quad.data.time), wall_s=wall)
