#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo closed loop: intercept the moving target, then hold standoff.

Single-line summary: reuse uav_mujoco_sim (Quadrotor + GeometricController)
to climb, fly to the predicted intercept point, then follow the ground
target at a fixed behind-distance/altitude; per-tick standoff error is
recorded and a naive "hover right above the target" baseline is provided
for comparison.
"""
from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np

from .follow import (CLIMB_SPEED, REF_TAU, TAIL_S,
                     FollowCfg, follow_reference, plan_intercept)

ATT_DAMPING = (-0.18, -0.18, -0.126)   # 姿态阻尼（与仓库其它闭环一致）
GROUND_Z = 0.05


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
class FollowResult:
    """闭环跟随结果。"""

    times: np.ndarray                 # (K,) 采样时刻
    quad_pos: np.ndarray              # (K, 3) 无人机真实位置
    tgt_pos: np.ndarray               # (K, 2) 目标位置
    follow_err: np.ndarray            # (K,) |真实位置 − 参考点|（跟随段）
    intercept_s: Optional[float]      # 截获时刻（None = 未截获）
    mean_err_m: float                 # 截获后稳态平均跟随误差
    max_err_m: float                  # 截获后最大跟随误差（含转角瞬态）
    turn_err_m: float                 # 转角附近（±4 s）最大误差
    stop_err_m: float                 # 目标停车瞬间的减速超调峰值
    final_err_m: float                # 目标停车后收尾段平均误差
    sim_s: float
    wall_s: float

    def as_dict(self) -> dict:
        return dict(
            simulated=True,
            intercept_s=round(self.intercept_s, 2) if self.intercept_s else None,
            follow_error_mean_m=round(self.mean_err_m, 2),
            follow_error_max_m=round(self.max_err_m, 2),
            turn_error_max_m=round(self.turn_err_m, 2),
            stop_transient_max_m=round(self.stop_err_m, 2),
            final_error_m=round(self.final_err_m, 2),
            sim_duration_s=round(self.sim_s, 1),
            wall_time_s=round(self.wall_s, 1))


def _turn_windows(target, margin: float = 4.0) -> List[tuple]:
    """每个折弯点的 (t_enter, t_exit) 时间窗（±margin 秒）。"""
    wins = []
    segs = target.segs
    t = 0.0
    for i in range(len(segs) - 1):
        t += segs[i][2] / target.speed
        wins.append((t - margin, t + margin))
    return wins


def follow_target(target, cfg: FollowCfg, start_xy=(0.0, 0.0),
                  dt: float = 0.01, record_every: int = 5,
                  progress: bool = False) -> FollowResult:
    """真飞一次"爬升 → 截获 → 跟随 → 停车收尾"。

    dt 必须等于 Quadrotor 控制周期（默认 0.01 s，内部 4 个 0.0025 s
    物理子步），每循环恰好推进一个控制周期，保证时钟与物理时间一致。
    跟随段参考由 follow_reference 给出（目标正后方 standoff 环上），
    参考速度 = 目标速度经一阶低通；目标状态用解析 pos_vel(t)，与
    ROS 节点将订阅到的目标里程计同一语义。
    """
    Quadrotor, GeometricController, model_dir = _load_backend()
    model_path = str(Path(model_dir) / "quadrotor.xml")

    t_end = target.duration_s + TAIL_S + 4.0
    n_steps = int(round(t_end / dt))
    alpha = min(1.0, dt / REF_TAU)
    climb_s = cfg.alt_m / CLIMB_SPEED

    quad = Quadrotor(model_path)
    quad.reset((start_xy[0], start_xy[1], GROUND_Z))
    ctrl = GeometricController(quad.params, kw=ATT_DAMPING)

    # 截获段参考：按解析截获方案生成时间参数化直线——从爬升终点以巡航
    # 速度直飞 t_i 时刻的 standoff 点（由固定点迭代保证该点在 t_i 恰好
    # 可达）。直接把远处移动参考点喂给几何控制器会顶到倾角/推力上限，
    # 实测俯冲掉高 20 m、水平速度 16 m/s。
    plan = plan_intercept(target, start_xy, cfg)
    p_start = np.asarray(start_xy, dtype=float)[:2]
    if plan["feasible"]:
        t_i = float(plan["t_intercept_s"])
        p_i, _v_i, _d_i = follow_reference(*target.pos_vel(t_i), cfg)
        u_int = p_i[:2] - p_start
        seg_len = float(np.linalg.norm(u_int))
        u_int = u_int / max(seg_len, 1e-9)
        v_int = seg_len / max(t_i - climb_s, 1e-9)
    else:
        # 追不上：仍按当前方向以巡航速度追，截获时刻记为 None
        p_t0, v_t0 = target.pos_vel(0.0)
        p_i, _v_i, _d_i = follow_reference(p_t0, v_t0, cfg)
        u_int = p_i[:2] - p_start
        seg_len = float(np.linalg.norm(u_int))
        u_int = u_int / max(seg_len, 1e-9)
        v_int = cfg.cruise_speed
        t_i = 1e9

    times, q_log, t_log, e_log = [], [], [], []
    v_filt = np.zeros(3)
    captured = False
    capture_t: Optional[float] = None
    prev_dir: Optional[np.ndarray] = None
    turn_wins = _turn_windows(target)
    t0_wall = time.time()

    for step in range(n_steps):
        t = step * dt
        p_tgt, v_tgt = target.pos_vel(t)

        if t < climb_s:
            # 垂直爬升段
            p_ref = np.array([start_xy[0], start_xy[1], cfg.alt_m])
            frac = min(t / max(climb_s, 1e-9), 1.0)
            p_ref[2] = GROUND_Z + (cfg.alt_m - GROUND_Z) * frac
            v_ref = np.array([0.0, 0.0, CLIMB_SPEED if frac < 1.0 else 0.0])
        elif not captured:
            # 截获段：沿规划直线以巡航速度推进
            s = min((t - climb_s) * v_int, seg_len)
            p_ref = np.array([p_start[0] + u_int[0] * s,
                              p_start[1] + u_int[1] * s, cfg.alt_m])
            v_ref = np.array([u_int[0] * v_int, u_int[1] * v_int, 0.0])
            if t >= t_i or s >= seg_len - 1e-6:
                captured = True
                capture_t = t
                prev_dir = None
        else:
            # 跟随段：目标正后方 standoff 环
            p_ref, v_ref, d_off = follow_reference(p_tgt, v_tgt, cfg, prev_dir, dt)
            prev_dir = d_off

        if alpha < 1.0:
            v_filt += (v_ref - v_filt) * alpha
            v_use = v_filt.copy()
        else:
            v_use = v_ref.copy()
        p_use = p_ref.copy()
        p_use[2] = max(p_use[2], GROUND_Z)
        # 期望偏航：面向目标（观测/拍摄方向）
        yaw_d = float(np.arctan2(p_tgt[1] - quad.qpos[1],
                                 p_tgt[0] - quad.qpos[0]))
        thrusts = ctrl.act(quad, p_use, v_use, np.zeros(3), yaw_d)
        quad.step_with_control(thrusts, 1)

        if step % record_every == 0:
            err = float(np.linalg.norm(quad.qpos[:3] - p_ref))
            times.append(t)
            q_log.append(quad.qpos[:3].copy())
            t_log.append(np.asarray(p_tgt, dtype=float).copy())
            if captured:
                e_log.append((t, err))

        if progress and step % 5000 == 0:
            print("  [flyer] sim t=%.1f / %.1f s captured=%s" % (t, t_end, captured))

    times.append(n_steps * dt)
    q_log.append(quad.qpos[:3].copy())
    t_log.append(np.asarray(target.pos_vel(t_end)[0]).copy())

    err = np.asarray([e for _t, e in e_log], dtype=float) if e_log \
        else np.zeros(1)
    ts = np.asarray([t_ for t_, _e in e_log], dtype=float) if e_log \
        else np.zeros(1)
    # 稳态：截获后 3 s 起、目标仍在行驶的样本（目标停车瞬间速度前馈
    # 从 6 m/s 骤降到 0，滤波滞后会产生一次 3~4 m 的减速超调，单独统计）
    moving = ts < target.duration_s
    settled = (ts >= (capture_t or 0.0) + 3.0) & moving
    in_turn = np.zeros(len(ts), dtype=bool) if len(ts) else np.zeros(1, dtype=bool)
    for lo, hi in turn_wins:
        in_turn |= (ts >= lo) & (ts <= hi)
    tail0 = target.duration_s
    in_tail = ts >= tail0

    return FollowResult(
        times=np.asarray(times),
        quad_pos=np.asarray(q_log),
        tgt_pos=np.asarray(t_log),
        follow_err=err,
        intercept_s=capture_t,
        mean_err_m=float(np.mean(err[settled])) if np.any(settled) else float(err.mean()),
        max_err_m=float(err[settled].max()) if np.any(settled) else float(err.max()),
        turn_err_m=float(err[settled & in_turn].max())
        if np.any(settled & in_turn) else float(err[settled].max())
        if np.any(settled) else float(err.max()),
        stop_err_m=float(err[~(ts < target.duration_s) & ~in_tail].max())
        if np.any(~(ts < target.duration_s) & ~in_tail) else 0.0,
        final_err_m=float(err[in_tail].mean()) if np.any(in_tail)
        else float(err[-1]),
        sim_s=n_steps * dt, wall_s=time.time() - t0_wall)


def naive_above_target(target, cfg: FollowCfg, result: FollowResult,
                       dt: float = 0.01) -> float:
    """朴素基线的跟随误差：参考点取目标正上方（不做 standoff 后移）。

    用同一条真实轨迹 result.quad_pos 对比"目标正上方 alt_m"这一朴素
    参考的误差——转角处正上方参考会随目标急转，误差峰值显著更大。
    返回与 FollowResult 同口径的转角段最大误差（m）。
    """
    errs, ts = [], []
    for k, t in enumerate(result.times):
        p_tgt, _v = target.pos_vel(float(t))
        ref = np.array([p_tgt[0], p_tgt[1], cfg.alt_m])
        errs.append(float(np.linalg.norm(result.quad_pos[k] - ref)))
        ts.append(float(t))
    errs = np.asarray(errs)
    ts = np.asarray(ts)
    after = ts >= (result.intercept_s or 0.0) + 3.0
    in_turn = np.zeros(len(ts), dtype=bool)
    for lo, hi in _turn_windows(target):
        in_turn |= (ts >= lo) & (ts <= hi)
    sel = errs[after & in_turn]
    return float(sel.max()) if sel.size else float(errs[after].max())
