#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 闭环：按观察站飞一圈，在每站悬停采样，用真实位姿定位。

和解析模式的分工：解析模式相信"相机就在设定的点上、姿态是水平的"；
这里所有位置/姿态都来自刚体积分，悬停时的倾角、位置漂移全都真实存在。
两种模式共用 ``analysis`` 里的同一套统计函数与 ``fusion`` 里的同一个交会解，
不存在"仿真里另算一套"。

参考信号用带速度/加速度上限的直线段生成器，而不是直接把目标点丢给位置环：
80 m 的阶跃指令会让推力项顶到限幅、机体姿态翻转，那条轨迹没有工程意义。
"""
from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .analysis import (TILT_SIGMA_DEG, _rotate_small, from_station_directions,
                       single_station_error)
from .cases import GeoCase
from .fusion import angular_spread_deg, mean_direction, summarize
from .geometry import los_to, unit

#: 解算用的地面高度（ENU，米）——全模块唯一口径：解析、闭环、ROS 节点都用它，
#: 与 ``geometry.Ray.ground_hit`` 的默认值保持一致，不再各写一个 0.0 / 0.05
GROUND_Z = 0.0
#: 起飞点相对地面的高度（米）。四旋翼模型的原点不在机体中心，贴地起步第一帧
#: 就会穿地，所以给 5 cm 起步余量——它只是初始条件，不参与任何解算
SPAWN_CLEARANCE_M = 0.05
ATT_DAMPING = (-0.18, -0.18, -0.126)
#: 转场巡航速度与加减速上限（m/s, m/s²）
CRUISE_SPEED = 6.0
CRUISE_ACCEL = 2.0
#: 悬停判定：位置误差与速度都要在这个门限内
HOVER_TOL_M = 1.0
HOVER_SPEED = 0.30
#: 每站最多等多久（s）
STATION_TIMEOUT_S = 90.0
#: 采样步长（仿真步长 0.01 s，每 2 步取一帧 ⇒ 50 Hz）
SAMPLE_STRIDE = 2


def _load_backend():
    """导入课程仿真底座；源码目录直接跑时自动挂同级包路径。"""
    try:
        from uav_mujoco_sim.core.quadrotor import (  # type: ignore
            GeometricController, Quadrotor, quat_to_mat)
        from uav_mujoco_sim.core.paths import quadrotor_xml  # type: ignore
        return Quadrotor, GeometricController, quat_to_mat, quadrotor_xml()
    except Exception:                                     # noqa: BLE001
        here = Path(__file__).resolve()
        sibling = here.parents[3] / "uav_mujoco_sim"
        if sibling.is_dir() and str(sibling) not in sys.path:
            sys.path.insert(0, str(sibling))
        from uav_mujoco_sim.core.quadrotor import (  # type: ignore
            GeometricController, Quadrotor, quat_to_mat)
        from uav_mujoco_sim.core.paths import quadrotor_xml  # type: ignore
        return Quadrotor, GeometricController, quat_to_mat, quadrotor_xml()


class _LineRef:
    """直线段参考：梯形速度曲线，位置/速度/加速度都给到控制器。"""

    def __init__(self, start: Sequence[float], goal: Sequence[float],
                 v_max: float = CRUISE_SPEED, a_max: float = CRUISE_ACCEL):
        self.start = np.asarray(start, dtype=float).copy()
        self.goal = np.asarray(goal, dtype=float).copy()
        d = self.goal - self.start
        self.length = float(np.linalg.norm(d))
        self.u = d / self.length if self.length > 1e-9 else np.zeros(3)
        self.v_max = float(v_max)
        self.a_max = float(a_max)
        self.s = 0.0
        self.v = 0.0

    def step(self, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        if self.length <= 1e-9:
            return self.goal.copy(), np.zeros(3), np.zeros(3)
        remain = max(0.0, self.length - self.s)
        v_allow = math.sqrt(max(0.0, 2.0 * self.a_max * remain))
        v_t = min(self.v_max, v_allow)
        if self.v < v_t:
            self.v = min(v_t, self.v + self.a_max * dt)
            a = self.a_max
        elif self.v > v_t:
            self.v = max(v_t, self.v - self.a_max * dt)
            a = -self.a_max
        else:
            a = 0.0
        self.s = min(self.length, self.s + self.v * dt)
        return self.start + self.u * self.s, self.u * self.v, self.u * a

    @property
    def done(self) -> bool:
        return self.length <= 1e-9 or (self.length - self.s) < 1e-3


@dataclass
class StationVisit:
    """一次观察站的到访记录。"""

    index: int
    target: np.ndarray
    pos_mean: Optional[np.ndarray] = None
    hover_error_m: float = float("nan")
    bias_deg: Tuple[float, float] = (0.0, 0.0)
    spread_deg: float = float("nan")
    tilt_mean_deg: Tuple[float, float] = (0.0, 0.0)
    mean_dir_true: Optional[np.ndarray] = None
    mean_dir_level: Optional[np.ndarray] = None
    single_errors: List[float] = field(default_factory=list)
    level_errors: List[float] = field(default_factory=list)
    n_samples: int = 0


@dataclass
class FlyResult:
    """整个架次的输出，字段与解析模式对齐。"""

    case: str
    truth: np.ndarray
    visits: List[StationVisit]
    trajectory: List[np.ndarray] = field(default_factory=list)
    sim_s: float = 0.0
    wall_s: float = 0.0
    max_hover_error_m: float = float("nan")


def fly_case(case: GeoCase, dt: float = 0.01, seed: int = 3) -> FlyResult:
    """让无人机依次飞到各观察站、悬停采样，返回真实位姿下的测量。"""
    Quadrotor, GeometricController, quat_to_mat, xml = _load_backend()
    quad = Quadrotor(xml, dt=dt)
    ctrl = GeometricController(quad.params, kw=ATT_DAMPING)

    # 悬停时的水平外力：定点保持要抵消它，机体就会倾转几度。"拿指令姿态
    # 当真实姿态"的对照正是靠这个倾角产生误差。注意 xfrc_applied 在 MuJoCo
    # 里跨步保留，所以每步改写一次力分量，不能逐子步累加——那会按子步数叠乘。
    wind_force = np.array([float(case.wind_n[0]), float(case.wind_n[1]), 0.0])

    def _advance(thrusts: np.ndarray) -> None:
        quad.data.xfrc_applied[quad.body_id, 0:3] = wind_force
        quad.step_with_control(thrusts, 1)

    truth = np.asarray(case.target, dtype=float)
    first = case.stations[0].position.copy()
    quad.reset((float(first[0]), float(first[1]),
                GROUND_Z + SPAWN_CLEARANCE_M))

    rng = np.random.default_rng(seed)
    visits: List[StationVisit] = []
    traj: List[np.ndarray] = []
    t0 = time.time()
    max_hover_err = 0.0
    step_count = 0

    def _track(ref: _LineRef, on_sample=None) -> None:
        nonlocal max_hover_err, step_count
        settled = 0
        need = int(round(1.0 / dt))
        t_begin = quad.time
        while quad.time - t_begin < STATION_TIMEOUT_S:
            p_d, v_d, a_d = ref.step(dt)
            thrusts = ctrl.act(quad, p_d, v_d, a_d, 0.0)
            _advance(thrusts)
            step_count += 1
            if step_count % 5 == 0:
                traj.append(quad.state().pos.copy())
            st = quad.state()
            err = float(np.linalg.norm(st.pos - ref.goal))
            if ref.done and err > max_hover_err:
                max_hover_err = err
            if ref.done and err < HOVER_TOL_M and \
                    float(np.linalg.norm(st.vel)) < HOVER_SPEED:
                settled += 1
            else:
                settled = 0
            if on_sample is not None:
                on_sample(st, step_count)
            if settled >= need:
                return
        raise RuntimeError(
            f"飞到观察站 {np.round(ref.goal, 1)} 超时"
            f"（{STATION_TIMEOUT_S:.0f} s 未稳定）")

    for idx, st_def in enumerate(case.stations):
        goal = st_def.position.copy()
        start = quad.state().pos.copy()
        _track(_LineRef(start, goal))

        bx = float(rng.normal(0.0, case.bias_deg))
        by = float(rng.normal(0.0, case.bias_deg))
        visit = StationVisit(index=idx, target=goal, bias_deg=(bx, by))
        dirs_true: List[np.ndarray] = []
        dirs_level: List[np.ndarray] = []
        pos_list: List[np.ndarray] = []
        tilts: List[Tuple[float, float]] = []
        want = int(case.samples)

        def _collect(st, _n) -> None:
            if len(dirs_true) >= want or _n % SAMPLE_STRIDE:
                return
            R = quat_to_mat(st.quat)
            los_true = los_to(truth, st.pos)
            los_body = R.T @ los_true
            jx, jy = rng.normal(0.0, case.sigma_deg, 2)
            meas_body = _rotate_small(los_body, bx + jx, by + jy)
            dirs_true.append(unit(R @ meas_body))
            a = st.yaw
            R_yaw = np.array([[math.cos(a), -math.sin(a), 0.0],
                              [math.sin(a), math.cos(a), 0.0],
                              [0.0, 0.0, 1.0]])
            dirs_level.append(unit(R_yaw @ meas_body))
            pos_list.append(st.pos.copy())
            tilts.append((float(st.roll), float(st.pitch)))

        _track(_LineRef(goal, goal))
        # 稳定之后按固定时长悬停采样（悬停时仍走同一套控制，不是"冻结物理"）
        for k in range(want * SAMPLE_STRIDE):
            thrusts = ctrl.act(quad, goal, np.zeros(3), np.zeros(3), 0.0)
            _advance(thrusts)
            step_count += 1
            if step_count % 5 == 0:
                traj.append(quad.state().pos.copy())
            _collect(quad.state(), k)
        if len(dirs_true) < want:
            raise RuntimeError(f"第 {idx} 站只采到 {len(dirs_true)} / {want} 帧")

        visit.n_samples = len(dirs_true)
        visit.pos_mean = np.mean(np.asarray(pos_list), axis=0)
        visit.hover_error_m = float(np.linalg.norm(visit.pos_mean - goal))
        visit.mean_dir_true = mean_direction(dirs_true)
        visit.mean_dir_level = mean_direction(dirs_level)
        visit.spread_deg = angular_spread_deg(dirs_true)
        visit.tilt_mean_deg = (float(np.degrees(np.mean([t[0] for t in tilts]))),
                              float(np.degrees(np.mean([t[1] for t in tilts]))))
        for p, d in zip(pos_list, dirs_true):
            e = single_station_error(p, d, truth, ground_z=GROUND_Z)
            if e is not None:
                visit.single_errors.append(e)
        for p, d in zip(pos_list, dirs_level):
            e = single_station_error(p, d, truth, ground_z=GROUND_Z)
            if e is not None:
                visit.level_errors.append(e)
        visits.append(visit)

    return FlyResult(case=case.name, truth=truth, visits=visits,
                     trajectory=traj, sim_s=float(quad.time),
                     wall_s=time.time() - t0, max_hover_error_m=max_hover_err)


def summarize_flight(case: GeoCase, res: FlyResult,
                     tilt_sigma_deg: float = TILT_SIGMA_DEG) -> Dict[str, object]:
    """把一次架次折算成与解析模式同口径的指标字典。

    口径对齐的关键：解析模式里的"单站误差"是**该站平均视线**打点的误差，
    所以这里也用每站平均视线算一份；逐帧误差单独放 ``per_sample``，
    它主要反映视轴抖动（会被平均掉），两者不是一回事。
    """
    stations = [v.pos_mean for v in res.visits]
    dirs = [v.mean_dir_true for v in res.visits]
    geo = from_station_directions(stations, dirs, res.truth, ground_z=GROUND_Z)

    single_all: List[float] = []
    level_all: List[float] = []
    for v in res.visits:
        single_all.extend(v.single_errors)
        level_all.extend(v.level_errors)

    ref = res.visits[0]
    ref_single = single_station_error(ref.pos_mean, ref.mean_dir_true, res.truth,
                                      ground_z=GROUND_Z)
    ref_level = single_station_error(ref.pos_mean, ref.mean_dir_level, res.truth,
                                     ground_z=GROUND_Z)

    if len(stations) > 1:
        station_fused = {
            "fused_m": geo["fused_m"],
            "ok": bool(geo["fused_ok"]),
            "reason": geo["fused_reason"],
            "intersection_deg": geo["intersection_deg"],
            "condition": geo["condition"],
            "residual_m": geo["residual_m"],
        }
    else:
        # 单站没有交会：把这一站平均视线的打点结果放到交会列，标注清楚
        station_fused = {
            "fused_m": geo["per_station_m"][0],
            "ok": geo["per_station_m"][0] is not None,
            "reason": "单站，无交会（取该站平均视线打点）",
            "intersection_deg": None,
            "condition": None,
            "residual_m": None,
        }
    stat_rows = []
    for v, mean_dir_err in zip(res.visits, geo["per_station_m"]):
        lvl = single_station_error(v.pos_mean, v.mean_dir_level, res.truth,
                                   ground_z=GROUND_Z)
        stat_rows.append({
            "index": v.index,
            "goal_m": [round(float(x), 3) for x in v.target],
            "hover_mean_m": [round(float(x), 3) for x in v.pos_mean],
            "hover_error_m": round(float(v.hover_error_m), 3),
            "hover_tilt_deg": [round(float(v.tilt_mean_deg[0]), 3),
                               round(float(v.tilt_mean_deg[1]), 3)],
            "los_bias_deg": [round(float(v.bias_deg[0]), 3),
                             round(float(v.bias_deg[1]), 3)],
            "los_spread_deg": round(float(v.spread_deg), 3),
            "n_samples": int(v.n_samples),
            "mean_dir_m": mean_dir_err,
            "per_sample_mean_m": round(float(np.mean(v.single_errors)), 4)
            if v.single_errors else None,
            "level_mean_dir_m": None if lvl is None else round(float(lvl), 4),
        })

    return {
        # 与解析模式同口径：0 号站平均视线的打点误差
        "single_ref_m": None if ref_single is None else round(float(ref_single), 4),
        "level_ref_m": None if ref_level is None else round(float(ref_level), 4),
        # 逐帧误差（含视轴抖动），只作参考
        "per_sample": {
            "single": summarize(single_all) if single_all else None,
            "level_assumption": summarize(level_all) if level_all else None,
        },
        "station_fused": station_fused,
        "per_station_m": geo["per_station_m"],
        "stations": stat_rows,
    }
