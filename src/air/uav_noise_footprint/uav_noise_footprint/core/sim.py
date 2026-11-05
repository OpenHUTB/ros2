#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 闭环复飞：优化前后航迹各抽一架真飞，用真实高度重算噪声。

降噪结论都发生在"计划空间"里，要可信必须回答两个问题：

1. **改了爬升角/巡航高度之后还飞得动吗？** 用课程既有的 ``uav_mujoco_sim``
   底座（同一份 ``quadrotor.xml`` 刚体模型 + 几何控制器）把剖面实例化，
   记录刚体积分出来的真实位置，统计全程最大跟踪误差（要求 < 8 m）；
2. **闭环数据是不是纸面值？** 把真实位置序列（20 Hz）直接喂回声学模型
   重新计算敏感点 LAeq——它与计划值的偏差就量化了"纸面 vs 真飞"。

实测参考：单机 40 s 仿真墙钟约 6 s，纯 CPU。
"""
from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

from .track import Flight

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
        # src/air/uav_noise_footprint/uav_noise_footprint/core/sim.py → src/air
        sibling = here.parents[3] / "uav_mujoco_sim"
        if sibling.is_dir() and str(sibling) not in sys.path:
            sys.path.insert(0, str(sibling))
        from uav_mujoco_sim.core.quadrotor import (  # type: ignore
            GeometricController, Quadrotor)
        from uav_mujoco_sim.core.paths import models_dir  # type: ignore
        return Quadrotor, GeometricController, models_dir()


@dataclass
class SimResult:
    """单机真飞记录。"""

    uid: str
    flight: Flight
    times: np.ndarray                    # (K,) 记录时刻
    pos: np.ndarray                      # (K, 3) 真实位置（刚体积分）
    final_error: float                   # 终点水平位置误差（m）
    max_track_error: float               # 全程最大跟踪误差（m）
    steps: int
    wall_time: float

    def to_dict(self) -> dict:
        return dict(uid=self.uid, steps=self.steps,
                    wall_time=round(self.wall_time, 2),
                    final_error=round(self.final_error, 2),
                    max_track_error=round(self.max_track_error, 2),
                    duration_sim=round(float(self.times[-1]), 1))


def run_flight(flight: Flight, dt: float = 0.01, substeps: int = 4,
               record_every: int = 5, tail: float = 1.0,
               max_steps: int = 200000, ref_tau: float = REF_TAU) -> SimResult:
    """让一架无人机按剖面真飞。

    参考速度前馈经一阶低通（``ref_tau``）。分段线性剖面在爬升→巡航→下降
    的每个衔接点都有速度方向变化，直接当年馈会激励姿态回路；真实飞控本来
    就有轨迹平滑器，这里按同一物理约束补上（与参考实现 uav_deconflict
    的闭环写法一致）。

    Parameters
    ----------
    dt
        控制周期（s），模型 ``timestep`` 为 0.0025，``substeps=4`` 对应 0.01 s。
    record_every
        每多少个控制步记录一次位置（5 → 20 Hz）。
    """
    Quadrotor, GeometricController, model_dir = _load_backend()
    model_path = os.path.join(model_dir, "quadrotor.xml")

    quad = Quadrotor(model_path)
    x0, y0 = flight.path[0]
    quad.reset((x0, y0, 0.05))
    ctrl = GeometricController(quad.params, kw=ATT_DAMPING)

    t_end = flight.end_time + tail
    n_steps = min(max_steps, int(round(t_end / dt)))
    times: List[float] = []
    pos: List[np.ndarray] = []
    track_err: List[float] = []
    v_filt = np.zeros(3)
    alpha = min(1.0, dt / ref_tau) if ref_tau and ref_tau > 0 else 1.0

    t0_wall = time.time()
    for step in range(n_steps):
        t = step * dt
        target = flight.hold_position(t)
        vel = flight.velocity_at(t)
        if vel is None:
            vel = np.zeros(3)
        if alpha < 1.0:
            v_filt += (vel - v_filt) * alpha
            vel = v_filt
        target = target.copy()
        target[2] = max(target[2], 0.05)
        thrusts = ctrl.act(quad, target, vel, np.zeros(3), 0.0)
        quad.step_with_control(thrusts, substeps)
        if step % record_every == 0:
            times.append(t)
            pos.append(quad.qpos[:3].copy())
            if flight.is_active(t):
                ref = flight.position_at(t)
                track_err.append(float(np.linalg.norm(quad.qpos[:3] - ref)))

    times.append(n_steps * dt)
    pos.append(quad.qpos[:3].copy())

    arr = np.asarray(pos, dtype=float)
    xe, ye = flight.path[-1]
    final_err = float(np.linalg.norm(arr[-1, :2] - np.array([xe, ye])))
    return SimResult(uid=flight.uid, flight=flight,
                     times=np.asarray(times, dtype=float), pos=arr,
                     final_error=final_err,
                     max_track_error=max(track_err) if track_err else 0.0,
                     steps=n_steps, wall_time=time.time() - t0_wall)


def sim_position_log(result: SimResult) -> Sequence[np.ndarray]:
    """取飞行活跃时段的真实位置序列（供噪声重算）。"""
    mask = (result.times >= result.flight.start_time - 1e-9) & \
           (result.times <= result.flight.end_time + 1e-9)
    return result.pos[mask], result.times[mask]
