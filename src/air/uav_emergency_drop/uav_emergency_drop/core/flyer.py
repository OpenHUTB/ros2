#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo closed loop: fly the drop route, release once per target.

Single-line summary: reuse uav_mujoco_sim (Quadrotor + GeometricController)
to fly over 3 drop targets, fire the release at the analytic ideal point,
record true release state and score predicted impact vs target.
"""
from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .ballistics import ReleasePlan, overflight_target
from .trigger import evaluate_drop

REF_TAU = 0.5                       # 参考速度一阶低通（s）
ATT_DAMPING = (-0.18, -0.18, -0.126)  # 姿态阻尼，抑制航向/分段阶跃激励
CLIMB_SPEED = 2.5                   # m/s
CUT_LEN = 15.0                      # 航路点切角长度（m）
GROUND_Z = 0.05
#: 桥梁通道半宽（m）：过河段的参考/实际轨迹必须落在
#: |x - bridge_x| <= 该值内。此前切角不受桥约束，参考轨迹在
#: x≈253.9 就进入河带（桥中心 260），通道宽度也没定义过。
BRIDGE_CHANNEL_HALF_M = 10.0


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


@dataclass
class DropRecord:
    """一次投放的完整记录（真实释放状态 + 解析落点）。"""

    target: np.ndarray          # 真实需求点
    overfly_wp: np.ndarray      # 航线越顶点（风补偿后）
    release_pos: np.ndarray     # MuJoCo 真实释放位置
    release_vel: np.ndarray     # 真实释放速度
    predicted_impact: np.ndarray
    error_m: float              # 预计落点与真实目标的水平偏差
    t_release: float

    def as_dict(self) -> dict:
        return dict(
            target_m=[round(float(self.target[0]), 2),
                      round(float(self.target[1]), 2)],
            overfly_waypoint_m=[round(float(self.overfly_wp[0]), 2),
                                round(float(self.overfly_wp[1]), 2)],
            release_pos_m=[round(float(v), 2) for v in self.release_pos[:2]],
            release_alt_m=round(float(self.release_pos[2]), 2),
            release_vel_mps=[round(float(v), 2) for v in self.release_vel[:2]],
            predicted_impact_m=[round(float(self.predicted_impact[0]), 2),
                                round(float(self.predicted_impact[1]), 2)],
            error_m=round(self.error_m, 2),
            t_release_s=round(self.t_release, 2))


@dataclass
class FlyResult:
    """闭环真飞结果。"""

    times: np.ndarray
    pos: np.ndarray                      # (K, 3) 真实位置
    route: List[Tuple[float, float]]
    targets: List[np.ndarray]
    releases: List[DropRecord]
    max_track_error_m: float             # 巡航段全程最大跟踪误差（含转角瞬态）
    max_track_error_settled_m: float     # 爬升过渡 4 s 后的巡航最大跟踪误差
    max_track_error_release_m: float     # 投放窗口附近最大跟踪误差
    final_hover_error_m: float           # 任务末端悬停最后 0.8 s 平均误差
    sim_s: float
    wall_s: float

    def as_dict(self) -> dict:
        errs = [r.error_m for r in self.releases]
        return dict(
            simulated=True,
            route_m=[[round(x, 1), round(y, 1)] for x, y in self.route],
            releases=[r.as_dict() for r in self.releases],
            impact_error_mean_m=round(float(np.mean(errs)), 2) if errs else None,
            impact_error_max_m=round(float(np.max(errs)), 2) if errs else None,
            max_track_error_m=round(self.max_track_error_m, 2),
            max_track_error_settled_m=round(self.max_track_error_settled_m, 2),
            max_track_error_release_m=round(self.max_track_error_release_m, 2),
            final_hover_error_m=round(self.final_hover_error_m, 2),
            sim_duration_s=round(self.sim_s, 1),
            wall_time_s=round(self.wall_s, 1))


def build_route(case):
    """补给点 → 前 3 个投放点的越顶点序列（侧风时逆风偏移 drift）。

    河流 case 先飞到桥梁入口、经桥梁通道过河，再从桥梁出口就近排序
    目标点——避免在急弯后立刻进入投放段（实测 105° 转弯后 7 s 仍有
    振荡，落点偏差被放大到 26 m）。桥梁入口/出口两个航路点不做切角
    （否则参考轨迹会斜穿河带、偏离桥中心线），过河段恒沿桥中心线。
    返回 (wps, true_targets, target_idx, no_cut)，
    target_idx[i] 是第 i 个投放点越顶点在 wps 中的下标，no_cut 是
    禁止切角的航路点下标集合（过河测试用它复核通道约束）。
    """
    from .planner import route_order
    targets = case.top_targets(3)
    start = case.cargo.depot
    wps = [tuple(case.cargo.depot)]
    no_cut: set = set()
    if case.river is not None:
        wps += [(case.bridge_x, case.river[0]), (case.bridge_x, case.river[1])]
        no_cut = {len(wps) - 2, len(wps) - 1}
        start = (case.bridge_x, case.river[1])
    ordered = route_order(case.points, [p.pid for p in targets], start)
    pmap = {p.pid: p for p in targets}
    true_targets, target_idx = [], []
    for pid in ordered:
        t = pmap[pid].xy
        wp = overflight_target(t, case.plan)
        wps.append((float(wp[0]), float(wp[1])))
        true_targets.append(t)
        target_idx.append(len(wps) - 1)
    return wps, true_targets, target_idx, no_cut


def _build_legs(route: Sequence[Tuple[float, float]], h: float,
                lead_m: float, climb_speed: float = CLIMB_SPEED,
                cut_len: float = CUT_LEN,
                no_cut: (set, frozenset, type(None)) = None):
    """构建段表：垂直爬升段 + 巡航段，内部航路点做切角圆滑。

    急转弯处参考速度方向阶跃会激励位置环振荡（实测 120° 转角超调
    17 m），把转角拆成前后两个 ~Δ/2 的小弯：航路点 wp 换成
    wp − u_in·c 与 wp + u_out·c，c ≤ 0.8·lead 保证理想释放点仍落在
    进入段直线上。返回 (legs, drop_leg, drop_s)：drop_leg[i] 是第 i 个
    投放点所属段号，drop_s[i] 是段内理想释放弧长 s_ideal。
    """
    def _unit(a, b):
        d = np.array([b[0] - a[0], b[1] - a[1]], dtype=float)
        n = float(np.linalg.norm(d))
        return d / max(n, 1e-9), n

    # 切角后的巡航路径；记录每个原始航路点的原始进入段长度与切角量
    path: List[Tuple[float, float]] = [tuple(route[0])]
    in_len: List[float] = []
    cuts: List[float] = []
    for j in range(1, len(route)):
        u_in, l_in = _unit(route[j - 1], route[j])
        in_len.append(l_in)
        if j < len(route) - 1:
            u_out, l_out = _unit(route[j], route[j + 1])
            if no_cut and j in no_cut:
                c = 0.0     # 桥梁等通道航路点不切角，过河段沿中心线
            else:
                c = min(cut_len, 0.35 * l_in, 0.35 * l_out, 0.8 * lead_m)
            cuts.append(c)
            path.append((route[j][0] - u_in[0] * c, route[j][1] - u_in[1] * c))
            path.append((route[j][0] + u_out[0] * c, route[j][1] + u_out[1] * c))
        else:
            cuts.append(0.0)
            path.append(tuple(route[j]))

    legs = [dict(kind="climb",
                 a=np.array([route[0][0], route[0][1], GROUND_Z]),
                 b=np.array([route[0][0], route[0][1], h]),
                 length=h - GROUND_Z, u=np.array([0.0, 0.0, 1.0]))]
    for k in range(len(path) - 1):
        a = np.array([path[k][0], path[k][1], h])
        b = np.array([path[k + 1][0], path[k + 1][1], h])
        d = b - a
        length = float(np.linalg.norm(d))
        legs.append(dict(kind="cruise", a=a, b=b, length=length,
                         u=d / max(length, 1e-9)))

    # 每个航路点 j≥1 的进入段：切角时进入段终点 = wp − u_in·c，理想释放
    # 点 = wp − lead·u_in（仍在该段直线上，因 c ≤ 0.8·lead）
    drop_leg, drop_s = [], []
    for j in range(1, len(route)):
        c = cuts[j - 1]
        u_in, _ = _unit(route[j - 1], route[j])
        end_pt = np.array([route[j][0] - u_in[0] * c,
                           route[j][1] - u_in[1] * c])
        best, bestd = None, 1e18
        for li, leg in enumerate(legs):
            if leg["kind"] != "cruise":
                continue
            d = float(np.linalg.norm(leg["b"][:2] - end_pt))
            if d < bestd:
                bestd, best = d, li
        drop_leg.append(best)
        # 段起点是上一转角的切出点（已越过上一航路点 c_prev），需扣除
        c_prev = cuts[j - 2] if j >= 2 else 0.0
        drop_s.append(max(0.0, in_len[j - 1] - c_prev - lead_m))
    return legs, drop_leg, drop_s


def fly_drop_route(route: Sequence[Tuple[float, float]],
                   true_targets: Sequence[np.ndarray],
                   plan: ReleasePlan,
                   target_idx: Optional[Sequence[int]] = None,
                   dt: float = 0.01, record_every: int = 5,
                   tail: float = 6.0, progress: bool = False,
                   no_cut: Optional[set] = None) -> FlyResult:
    """按航线真飞并在每个投放点的可行时机投一次。

    target_idx[i] 是第 i 个投放点越顶点在 route 中的下标（默认 1..n，
    航线无绕行插入点时与此一致）。释放判据：与 ROS 节点共用
    evaluate_drop，用 MuJoCo 真实状态（位置/速度/高度）判定，预测落点
    进入接受半径一半或越过理想释放点后仍达标才放行（只投一次）。
    巡航段逐时刻统计 |真实位置 − 计划位置|。

    dt 必须等于 Quadrotor 控制周期（默认 0.01 s，内部 4 个 0.0025 s
    物理子步），每循环恰好推进一个控制周期，保证时钟与物理时间一致。
    """
    Quadrotor, GeometricController, model_dir = _load_backend()
    model_path = str(Path(model_dir) / "quadrotor.xml")

    legs, drop_leg, drop_s = _build_legs(route, plan.h, plan.lead_m,
                                         no_cut=no_cut)
    climb_s = legs[0]["length"] / CLIMB_SPEED
    if target_idx is None:
        target_idx = list(range(1, len(route)))

    t_end = climb_s + sum(l["length"] / plan.speed
                          for l in legs[1:]) + tail
    n_steps = int(round(t_end / dt))
    alpha = min(1.0, dt / REF_TAU)
    settle_t = climb_s + 4.0   # 爬升→巡航速度阶跃过渡（τ=0.5 低通约 3~4 s 收敛）

    quad = Quadrotor(model_path)
    quad.reset((route[0][0], route[0][1], GROUND_Z))
    ctrl = GeometricController(quad.params, kw=ATT_DAMPING)

    times, pos_log, err_log, err_rel_log = [], [], [], []
    releases: List[DropRecord] = []
    v_filt = np.zeros(3)
    seg_idx, seg_t0 = 0, 0.0
    released = [False] * len(true_targets)

    def planned_state(t: float):
        nonlocal seg_idx, seg_t0
        while seg_idx + 1 < len(legs):
            dur = (legs[seg_idx]["length"] / CLIMB_SPEED if seg_idx == 0
                   else legs[seg_idx]["length"] / plan.speed)
            if t - seg_t0 <= dur + 1e-9:
                break
            seg_t0 += dur
            seg_idx += 1
        leg = legs[seg_idx]
        speed = CLIMB_SPEED if leg["kind"] == "climb" else plan.speed
        s = min(max((t - seg_t0) * speed, 0.0), leg["length"])
        v_ref = leg["u"] * speed
        if seg_idx == len(legs) - 1 and s >= leg["length"] - 1e-9:
            v_ref = np.zeros(3)   # 末段终点悬停：参考速度归零，否则稳态偏差 kd/kp·v
        return leg, leg["a"] + leg["u"] * s, v_ref, s

    t0_wall = time.time()
    for step in range(n_steps):
        t = step * dt
        leg, p_ref, v_ref, s = planned_state(t)
        if alpha < 1.0:
            v_filt += (v_ref - v_filt) * alpha
            v_use = v_filt.copy()
        else:
            v_use = v_ref.copy()
        p_use = p_ref.copy()
        p_use[2] = max(p_use[2], GROUND_Z)
        thrusts = ctrl.act(quad, p_use, v_use, np.zeros(3), 0.0)
        quad.step_with_control(thrusts, 1)

        # 投放判定：与 ROS 节点共用 evaluate_drop，用 MuJoCo 真实状态
        # （位置/速度/高度）判定，落点校验不过不发指令（只投一次）。
        # 此前按"计划位置越过理想释放点"触发，闭环没有验证节点真正
        # 使用的判据链路。
        for i in range(len(true_targets)):
            if released[i]:
                continue
            k = target_idx[i] - 1  # drop_leg/drop_s 以 route[1..] 为序
            if seg_idx != drop_leg[k]:
                continue
            leg_k = legs[drop_leg[k]]
            verdict = evaluate_drop(quad.qpos[:3], quad.qvel[:3],
                                    np.asarray(leg_k["a"])[:2],
                                    np.asarray(leg_k["b"])[:2],
                                    true_targets[i], plan,
                                    released[i])
            if verdict.release_now:
                rp = quad.qpos[:3].copy()
                rv = quad.qvel[:3].copy()
                imp = np.asarray(verdict.predicted_impact, dtype=float)
                tgt = true_targets[i]
                releases.append(DropRecord(
                    target=np.asarray(tgt, dtype=float).copy(),
                    overfly_wp=np.array(route[target_idx[i]], dtype=float),
                    release_pos=rp, release_vel=rv,
                    predicted_impact=imp,
                    error_m=float(np.linalg.norm(imp - np.asarray(tgt)[:2])),
                    t_release=t))
                released[i] = True

        if step % record_every == 0:
            times.append(t)
            pos_log.append(quad.qpos[:3].copy())
            if leg["kind"] == "cruise":
                e = float(np.linalg.norm(quad.qpos[:3] - p_ref))
                err_log.append((t, e))
                near_window = any(
                    (not released[i])
                    and seg_idx == drop_leg[target_idx[i] - 1]
                    and abs(s - drop_s[target_idx[i] - 1]) < 15.0
                    for i in range(len(true_targets)))
                if near_window:
                    err_rel_log.append(e)

        if progress and step % 5000 == 0:
            print("  [flyer] sim t=%.1f / %.1f s" % (t, t_end))

    times.append(n_steps * dt)
    pos_log.append(quad.qpos[:3].copy())

    return FlyResult(
        times=np.asarray(times, dtype=float),
        pos=np.asarray(pos_log, dtype=float),
        route=[(float(a), float(b)) for a, b in route],
        targets=[np.asarray(t, dtype=float) for t in true_targets],
        releases=releases,
        max_track_error_m=max(e for _t, e in err_log) if err_log else 0.0,
        max_track_error_settled_m=max(
            (e for t_, e in err_log if t_ >= settle_t), default=0.0),
        max_track_error_release_m=max(err_rel_log) if err_rel_log else 0.0,
        final_hover_error_m=float(np.mean(
            [e for t_, e in err_log if t_ >= n_steps * dt - 0.8])
            ) if err_log else 0.0,
        sim_s=n_steps * dt, wall_s=time.time() - t0_wall)


def bridge_channel_deviation(legs, river, bridge_x,
                             half: float = BRIDGE_CHANNEL_HALF_M) -> float:
    """过河段相对桥中心线的最大横向偏差（m），用于通道约束自检。"""
    y_lo, y_hi = river
    worst = 0.0
    for leg in legs:
        if leg["kind"] != "cruise":
            continue
        a = np.asarray(leg["a"])[:2]
        b = np.asarray(leg["b"])[:2]
        d = b - a
        if abs(d[1]) < 1e-9:
            if y_lo <= a[1] <= y_hi:
                worst = max(worst, abs(float(a[0] - bridge_x)))
            continue
        for y_b in (y_lo, y_hi):
            t = (y_b - a[1]) / d[1]
            if 0.0 <= t <= 1.0:
                x = a[0] + t * d[0]
                worst = max(worst, abs(float(x - bridge_x)))
    return worst
