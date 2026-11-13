#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo closed loop: fly the search, measure RSSI, localize the beacon.

Single-line summary: reuse uav_mujoco_sim (Quadrotor + GeometricController)
to fly the expanding-square search at altitude, take quantized RSSI
readings from the true 3D range to a hidden beacon, then fly shrinking
measurement rings; the beacon position comes from LM trilateration over
the samples, and a final overflight verifies the estimate.
"""
from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from .locator import (SearchCfg, SearchState, expanding_square,
                      localize, ring_points)
from .rssi import RssiCfg

REF_TAU = 0.5                          # 参考速度一阶低通（s）
CLIMB_SPEED = 2.5                      # m/s
TAIL_S = 2.0                           # done/fail 后悬停收尾时长
GROUND_Z = 0.05
#: 到达测量点/航点的判定半径（水平）
ARRIVE_RADIUS_M = 2.0
#: 参考点领先无人机的最大距离：超过就把参考拽回机头前方，防止
#: 位置误差项顶到倾角上限（实测直接追远处航点会发散掉高）
REF_LEASH_M = 3.0
#: 允许的镜像翻转次数（单侧弧样本最多两个解，翻两次就该收手）
MAX_FLIPS = 2


def _load_backend():
    """导入课程仿真底座；源码目录直接运行时自动挂同级包路径。"""
    try:
        from uav_mujoco_sim.core.quadrotor import (  # type: ignore
            GeometricController, Quadrotor)
        from uav_mujoco_sim.core.paths import models_dir  # type: ignore
        return Quadrotor, GeometricController, models_dir()
    except Exception:                                    # noqa: BLE001
        here = Path(__file__).resolve()
        sibling = here.parents[3] / "uav_mujoco_sim"
        if sibling.is_dir() and str(sibling) not in sys.path:
            sys.path.insert(0, str(sibling))
        from uav_mujoco_sim.core.quadrotor import (  # type: ignore
            GeometricController, Quadrotor)
        from uav_mujoco_sim.core.paths import models_dir  # type: ignore
        return Quadrotor, GeometricController, models_dir()


def _in_area(est2, cfg: SearchCfg) -> bool:
    """估计点是否落在搜索区内（真信标一定在区内）。"""
    e = np.asarray(est2, dtype=float)[:2]
    return bool(0.0 <= e[0] <= cfg.area[0] and 0.0 <= e[1] <= cfg.area[1])


def _mirror_estimate(state: SearchState) -> np.ndarray:
    """把估计点沿样本主轴翻转，得到镜像解。

    检测发生在可检测半径边界，探圈往往只圈到一小段信号弧：近共线的
    样本让最小二乘在主轴两侧各有一个残差几乎一样的解（实测幽灵解
    偏 57 m、残差反而更小）。真解与镜像解关于主轴 u 对称，把估计点
    的法向分量取反即可换到另一侧。样本不足两点时原样返回。
    """
    est = np.asarray(state.estimate, dtype=float)[:2].copy()
    if len(state.samples) < 2:
        return est
    pts = np.asarray([np.asarray(s[0], dtype=float)[:2]
                      for s in state.samples])
    c = pts.mean(axis=0)
    _w, vec = np.linalg.eigh((pts - c).T @ (pts - c))
    u = vec[:, int(np.argmax(_w))]
    n = np.array([-u[1], u[0]])
    return est - 2.0 * float((est - c) @ n) * n


class _Leg:
    """时间参数化直线腿：参考从当前位置出发，以 speed 向目标推进。

    参考点最多领先无人机 REF_LEASH_M——超了就把参考拽回机头前方。
    位置环看到的误差始终 ≤ 3 m，控制器工作在线性区。
    """

    def __init__(self, start_pos, target, speed: float):
        self.p0 = np.asarray(start_pos, dtype=float)[:2].copy()
        self.target = np.asarray(target, dtype=float)[:2].copy()
        d = self.target - self.p0
        self.len = float(np.linalg.norm(d))
        self.u = d / max(self.len, 1e-9)
        self.speed = float(speed)
        self.t0: Optional[float] = None

    def ref(self, t: float, pos) -> Tuple[np.ndarray, np.ndarray, bool]:
        if self.t0 is None:
            self.t0 = t
        s = min((t - self.t0) * self.speed, self.len)
        p = self.p0 + self.u * s
        if float(np.linalg.norm(p - np.asarray(pos)[:2])) > REF_LEASH_M:
            # 落后太多：参考回拽到机头前方 1 m，重新起步。剩余长度按
            # 新起点到目标重算——不重算的话参考会冲过目标（实测采样
            # 点偏离标称圈上点 11 m，定位几何被破坏）。
            self.p0 = np.asarray(pos)[:2] + self.u * 1.0
            self.len = float(np.linalg.norm(self.target - self.p0))
            self.t0 = t
            s = min((t - self.t0) * self.speed, self.len)
            p = self.p0 + self.u * s
        done = (t - self.t0) * self.speed >= self.len
        v = self.u * self.speed if not done else np.zeros(2)
        return (np.array([p[0], p[1], 0.0]),
                np.array([v[0], v[1], 0.0]), done)


@dataclass
class Beacon:
    """隐藏信号源（只在仿真/验证里用，定位算法不知道它的位置）。"""

    xy: Tuple[float, float]
    z: float = 0.5                     # 天线离地高度

    def rssi_at(self, pos3, cfg: RssiCfg) -> float:
        d = float(np.linalg.norm(np.asarray(pos3, dtype=float)[:3]
                                 - np.array([self.xy[0], self.xy[1], self.z])))
        return cfg.rssi_at(d)


@dataclass
class LocateResult:
    """闭环搜索定位结果。"""

    times: np.ndarray
    pos: np.ndarray                     # (K, 3) 真实位置
    search_path: List[Tuple[float, float]]
    measure_points: List[np.ndarray]
    estimate: Optional[np.ndarray]
    beacon_xy: np.ndarray
    error_m: float                      # 最终估计与真实信标位置的水平偏差
    residual_m: float                   # 定位解的样本均方根残差
    n_measurements: int
    detected_t: Optional[float]
    est_t: List[float]                  # 每次估计更新的时刻
    est_xy: List[np.ndarray]            # 对应的估计点（逐轮）
    verified: bool                      # 复核点 RSSI 是否与估计自洽
    outcome: str                        # ok / not_found / no_fix
    sim_s: float
    wall_s: float

    def as_dict(self) -> dict:
        return dict(
            simulated=True,
            outcome=self.outcome,
            beacon_xy=[round(float(self.beacon_xy[0]), 2),
                       round(float(self.beacon_xy[1]), 2)],
            estimate_xy=[round(float(self.estimate[0]), 2),
                         round(float(self.estimate[1]), 2)]
            if self.estimate is not None else None,
            localization_error_m=round(self.error_m, 2)
            if self.estimate is not None else None,
            residual_m=self.residual_m,
            n_measurements=self.n_measurements,
            detected_at_s=round(self.detected_t, 2) if self.detected_t else None,
            verified=bool(self.verified),
            sim_duration_s=round(self.sim_s, 1),
            wall_time_s=round(self.wall_s, 1))


def locate_beacon(beacon: Beacon, cfg: SearchCfg,
                  rssi_cfg: RssiCfg = RssiCfg(),
                  start_xy=(0.0, 0.0), dt: float = 0.01,
                  record_every: int = 5, progress: bool = False,
                  beacon_z: float = 0.5) -> LocateResult:
    """真飞一次"扩张方形搜索 → 收缩测量圈 → 定位 → 复核"。

    dt 必须等于 Quadrotor 控制周期（默认 0.01 s，内部 4 个 0.0025 s
    物理子步）。RSSI 由信标真实 3D 距离经量化模型给出；定位算法只见
    (无人机位置, RSSI 读数)，不知道信标位置。
    """
    Quadrotor, GeometricController, model_dir = _load_backend()
    model_path = str(Path(model_dir) / "quadrotor.xml")

    state = SearchState(cfg=cfg)
    sq = expanding_square(cfg)
    quad = Quadrotor(model_path)
    quad.reset((start_xy[0], start_xy[1], GROUND_Z))
    ctrl = GeometricController(quad.params, kw=(-0.18, -0.18, -0.126))

    t_end = 900.0
    n_steps = int(round(t_end / dt))
    alpha = min(1.0, dt / REF_TAU)
    climb_s = cfg.alt_m / CLIMB_SPEED

    # 阶段：climb → square → ring（多轮）→ verify → done / fail
    phase = "climb"
    sq_i = 0
    leg: Optional[_Leg] = None
    ring_pts: List[np.ndarray] = []
    ring_i = 0
    v_filt = np.zeros(3)
    times, q_log = [], []
    detected_t: Optional[float] = None
    phase_since = 0.0
    prev_phase = phase
    _round_start = 0
    _d_min = float("inf")       # 逼近段见过的最小反演距离
    _d_min_t = 0.0
    _last_finite_t = 0.0
    _flips = 0
    est_t: List[float] = []
    est_xy: List[np.ndarray] = []

    def new_leg(target, speed: float) -> _Leg:
        return _Leg(quad.qpos[:3], target, speed)

    t0_wall = time.time()
    for step in range(n_steps):
        t = step * dt
        pos = quad.qpos[:3]

        if phase == "climb":
            p_ref = np.array([start_xy[0], start_xy[1], cfg.alt_m])
            frac = min(t / max(climb_s, 1e-9), 1.0)
            p_ref[2] = GROUND_Z + (cfg.alt_m - GROUND_Z) * frac
            v_ref = np.array([0.0, 0.0, CLIMB_SPEED if frac < 1.0 else 0.0])
            if t >= climb_s:
                leg = new_leg(sq[0], cfg.cruise_speed)
                phase = "square"
        elif phase == "square":
            p_ref, v_ref, done = leg.ref(t, pos)
            rssi = beacon.rssi_at(pos, rssi_cfg)
            if np.isfinite(rssi):
                # 检测：进入可检测半径，记样本并转入探圈
                detected_t = t
                state.detected_at = pos[:2].copy()
                state.samples.append((pos[:3].copy(), rssi))
                ring_pts = ring_points(pos[:2], cfg.explore_radius_m,
                                       cfg.explore_points)
                ring_i = 0
                phase = "explore"
            elif done:
                sq_i += 1
                if sq_i >= len(sq):
                    phase = "fail"          # 全区扫完仍未检测到
                else:
                    leg = new_leg(sq[sq_i], cfg.cruise_speed)
        elif phase == "explore":
            # 探圈：检测点附近小半径一圈。检测发生在可检测半径边界，
            # 圈的远端收不到信号——只用信号弧段上的样本拿粗估计。
            p_ref, v_ref, done = leg.ref(t, pos)
            if done:
                # 只在圈上采样：逐 tick 采样会产生上千个近共线样本，
                # 把最小二乘的几何拖垮
                rssi = beacon.rssi_at(pos, rssi_cfg)
                if np.isfinite(rssi):
                    state.samples.append((pos[:3].copy(), rssi))
                ring_i += 1
                if ring_i >= len(ring_pts):
                    out = localize(state.samples, rssi_cfg, beacon_z=beacon_z)
                    if out["feasible"]:
                        state.estimate = out["estimate"]
                        state.residual_m = out["residual_m"]
                        # 单侧弧样本的镜像歧义：两个解残差相近，但真信标
                        # 不可能落在搜索区外——出界的解直接镜像到主轴
                        # 另一侧；镜像后仍出界就夹回区内
                        for _ in range(2):
                            if _in_area(state.estimate, cfg):
                                break
                            state.estimate = _mirror_estimate(state)
                        if not _in_area(state.estimate, cfg):
                            state.estimate = np.clip(
                                state.estimate,
                                [1.0, 1.0],
                                [cfg.area[0] - 1.0, cfg.area[1] - 1.0])
                        leg = new_leg(state.estimate, cfg.cruise_speed)
                        phase = "approach"
                    else:
                        # 探圈样本不足：半径减半重飞一圈
                        cfg_r = cfg.explore_radius_m * 0.5
                        ring_pts = ring_points(pos[:2], cfg_r,
                                               cfg.explore_points)
                        ring_i = 0
                        leg = new_leg(ring_pts[0], cfg.cruise_speed)
                else:
                    leg = new_leg(ring_pts[ring_i], cfg.cruise_speed)
        elif phase == "approach":
            # 逼近：朝估计点飞，反演距离进入 approach_range 即转精圈
            p_ref, v_ref, done = leg.ref(t, pos)
            # 逼近段只读 RSSI 做距离触发，不记样本（沿线采样共线）
            rssi = beacon.rssi_at(pos, rssi_cfg)
            went_ring = False
            if np.isfinite(rssi):
                _last_finite_t = t
                d_inv = rssi_cfg.distance_from(rssi)
                if d_inv < _d_min:
                    _d_min = d_inv
                    _d_min_t = t
                elif (_flips < MAX_FLIPS and t - _d_min_t > 8.0
                      and d_inv > max(1.5 * _d_min, _d_min + 10.0)):
                    # 越飞信号越差：单侧弧样本存在镜像歧义——LS 在样本
                    # 主轴两侧各有一个解。把估计点沿样本主轴翻转再试。
                    _flips += 1
                    state.estimate = _mirror_estimate(state)
                    leg = new_leg(state.estimate, cfg.cruise_speed)
                    _d_min, _d_min_t = d_inv, t
                elif d_inv <= cfg.approach_range_m:
                    # 反演距离已进近场：就地拉精圈
                    ring_pts = ring_points(pos[:2], cfg.fine_radius0_m,
                                           cfg.ring_points)
                    ring_i = 0
                    state.round_no = 0
                    _round_start = len(state.samples)
                    leg = new_leg(ring_pts[0], cfg.cruise_speed)
                    phase = "refine"
                    went_ring = True
            elif (_flips < MAX_FLIPS and t - _last_finite_t > 12.0):
                # 逼近目标在可检测半径外、一路无信号：多半是镜像解，
                # 翻到主轴另一侧
                _flips += 1
                state.estimate = _mirror_estimate(state)
                leg = new_leg(state.estimate, cfg.cruise_speed)
                _last_finite_t = t
                _d_min, _d_min_t = float("inf"), t
            if not went_ring and done:
                # 到了估计点还没进入 approach_range（粗估计偏差大）：
                # 就地拉精圈——只要整圈在可检测半径内就有完整几何
                ring_pts = ring_points(pos[:2], min(cfg.fine_radius0_m,
                                                    cfg.approach_range_m),
                                       cfg.ring_points)
                ring_i = 0
                state.round_no = 0
                _round_start = len(state.samples)
                leg = new_leg(ring_pts[0], cfg.cruise_speed)
                phase = "refine"
        elif phase == "refine":
            # 精圈：以估计点为圆心收缩测量，整圈都有信号。每轮只用
            # 本轮样本定位——探圈的弧段样本几何差、残差大，混在一起
            # 会把最小二乘的解拖离真值（实测 20 个混合样本定位误差
            # 17 m，8 个纯精圈样本误差 <2 m）。
            p_ref, v_ref, done = leg.ref(t, pos)
            if done:
                rssi = beacon.rssi_at(pos, rssi_cfg)
                if np.isfinite(rssi):
                    state.samples.append((pos[:3].copy(), rssi))
                ring_i += 1
                if (ring_i >= len(ring_pts) and len(state.samples) == _round_start
                        and _flips < MAX_FLIPS):
                    # 整圈一个信号都没有：估计点在可检测半径外，
                    # 镜像回另一侧重新逼近
                    _flips += 1
                    state.estimate = _mirror_estimate(state)
                    if not _in_area(state.estimate, cfg):
                        state.estimate = np.clip(
                            state.estimate, [1.0, 1.0],
                            [cfg.area[0] - 1.0, cfg.area[1] - 1.0])
                    leg = new_leg(state.estimate, cfg.cruise_speed)
                    phase = "approach"
                    _last_finite_t = t
                    _d_min, _d_min_t = float("inf"), t
                elif ring_i >= len(ring_pts):
                    out = localize(state.samples[_round_start:], rssi_cfg,
                                   init=state.estimate, beacon_z=beacon_z)
                    if out["feasible"]:
                        state.estimate = out["estimate"]
                        state.residual_m = out["residual_m"]
                        state.round_no += 1
                        if state.round_no > cfg.max_refine_rounds:
                            leg = new_leg(state.estimate, cfg.cruise_speed)
                            phase = "verify"
                        else:
                            radius = cfg.fine_radius0_m * cfg.ring_shrink \
                                ** (state.round_no - 1)
                            ring_pts = ring_points(state.estimate, radius,
                                                   cfg.ring_points)
                            ring_i = 0
                            leg = new_leg(ring_pts[0], cfg.cruise_speed)
                    else:
                        # 精圈样本退化：半径减半重飞
                        radius = cfg.fine_radius0_m * cfg.ring_shrink \
                            ** state.round_no
                        ring_pts = ring_points(state.estimate, radius,
                                               cfg.ring_points)
                        ring_i = 0
                        leg = new_leg(ring_pts[0], cfg.cruise_speed)
                elif ring_i < len(ring_pts):
                    leg = new_leg(ring_pts[ring_i], cfg.cruise_speed)
        elif phase == "verify":
            p_ref, v_ref, done = leg.ref(t, pos)
            if done:
                state.samples.append((pos[:3].copy(),
                                      beacon.rssi_at(pos, rssi_cfg)))
                phase = "done"
        else:  # done / fail：原地悬停
            p_ref = pos.copy()
            v_ref = np.zeros(3)

        # 记录估计的演进：每轮定位更新一个点，画"误差随轮次收敛"用
        if state.estimate is not None and (
                not est_xy or not np.allclose(est_xy[-1], state.estimate)):
            est_t.append(t)
            est_xy.append(np.array(state.estimate, dtype=float).copy())

        if phase != prev_phase:
            phase_since = 0.0
            prev_phase = phase
        else:
            phase_since += dt

        if alpha < 1.0:
            v_filt += (v_ref - v_filt) * alpha
            v_use = v_filt.copy()
        else:
            v_use = v_ref.copy()
        p_use = np.asarray(p_ref, dtype=float).copy()
        p_use[2] = cfg.alt_m
        thrusts = ctrl.act(quad, p_use, v_use, np.zeros(3), 0.0)
        quad.step_with_control(thrusts, 1)

        if step % record_every == 0:
            times.append(t)
            q_log.append(quad.qpos[:3].copy())
        if progress and step % 10000 == 0:
            print("  [flyer] sim t=%.1f phase=%s" % (t, phase))
        if phase in ("done", "fail") and phase_since >= TAIL_S:
            break

    t_end_actual = t + dt
    times.append(t_end_actual)
    q_log.append(quad.qpos[:3].copy())
    pos_log = np.asarray(q_log)

    est = state.estimate
    err = float(np.linalg.norm(est - np.asarray(beacon.xy))) \
        if est is not None else float("inf")
    # 复核：在估计点实测的 RSSI 反演距离，应与"估计点即信标"自洽
    verified = False
    if est is not None:
        rssi_end = beacon.rssi_at(pos_log[-1], rssi_cfg)
        if np.isfinite(rssi_end):
            # 距离是三维的：复核点位于估计点正上方 20 m，只比水平
            # 距离会把 20 m 的高度差当成 0（永远对不上）
            d_pred = float(np.linalg.norm(
                pos_log[-1][:3] - np.array([est[0], est[1], beacon_z])))
            d_meas = rssi_cfg.distance_from(rssi_end)
            verified = abs(d_pred - d_meas) < max(2.5, 0.08 * d_pred)

    outcome = "ok" if est is not None else (
        "not_found" if phase == "fail" else "no_fix")

    return LocateResult(
        times=np.asarray(times), pos=pos_log,
        search_path=sq, measure_points=[s[0][:2] for s in state.samples],
        estimate=est, beacon_xy=np.asarray(beacon.xy, dtype=float),
        error_m=err, residual_m=state.residual_m or 0.0,
        n_measurements=len(state.samples), detected_t=detected_t,
        est_t=est_t, est_xy=est_xy,
        verified=verified, outcome=outcome, sim_s=t_end_actual,
        wall_s=time.time() - t0_wall)
