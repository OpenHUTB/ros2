#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多步态轨迹生成与平滑切换。

- 每个步态由周期 T、支撑相占比、步长、抬脚高度、髋高与躯干前倾定义；
- 摆动脚的位置由三次多项式（smoothstep / 16u^2(1-u)^2）生成；
- 步态切换时，用三次 Hermite 样条把当前关节角/角速度平滑过渡到目标步态，
  保证位置与速度连续（C1），并输出解析角速度用于 /joint_cmd 前馈。
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .kinematics import BipedModel
from .spline import hermite_vector, hermite_vector_derivative


@dataclass
class GaitSpec:
    name: str
    cycle: float
    stance: float
    step: float
    clearance: float
    hip_height: float
    bob: float
    lean: float

    @property
    def velocity(self) -> float:
        """前进速度：一个周期走两步，每步 step。"""
        if self.cycle <= 1e-9 or self.step <= 1e-9:
            return 0.0
        return 2.0 * self.step / self.cycle


class GaitPlanner:
    def __init__(self, model: BipedModel, gaits: Dict[str, GaitSpec],
                 schedule: List[Tuple[str, float]], transition_time: float = 0.6,
                 mode: str = "spline", stand_foot_offset: float = 0.10):
        if mode not in ("spline", "direct"):
            raise ValueError("mode 必须是 spline 或 direct")
        self.model = model
        self.gaits = gaits
        self.transition_time = float(transition_time)
        self.mode = mode
        self.stand_offset = float(stand_foot_offset)

        self.schedule: List[Tuple[float, str]] = []
        t = 0.0
        for name, duration in schedule:
            self.schedule.append((t, name))
            t += float(duration)
        self.total_time = t

        self.base_x = 0.0
        self.base_v = 0.0
        self.gait_name = self.schedule[0][1]
        self.gait_start = 0.0
        self.transition: Optional[dict] = None
        self.last_q = self._evaluate(self.gait_name, 0.0)
        self.last_dq = np.zeros(6)
        self.last_t = 0.0

    def gait_at(self, t: float) -> str:
        name = self.schedule[0][1]
        for start, gait in self.schedule:
            if t >= start:
                name = gait
            else:
                break
        return name

    def _foot_relative(self, gait: GaitSpec, side: str, phase: float
                       ) -> Tuple[float, float, bool]:
        p = phase if side == "left" else (phase + 0.5) % 1.0
        if gait.step <= 1e-9:
            offset = self.stand_offset
            x_rel = offset if side == "left" else -offset
            return x_rel, 0.0, True
        amp = gait.stance * gait.step
        if p < gait.stance:
            x_rel = amp - 2.0 * amp * (p / gait.stance)
            return x_rel, 0.0, True
        u = (p - gait.stance) / (1.0 - gait.stance)
        smooth = 3.0 * u ** 2 - 2.0 * u ** 3
        x_rel = -amp + 2.0 * amp * smooth
        bump = 16.0 * u ** 2 * (1.0 - u) ** 2
        return x_rel, gait.clearance * bump, False

    def _evaluate(self, gait_name: str, t_local: float,
                  base_x: Optional[float] = None) -> np.ndarray:
        gait = self.gaits[gait_name]
        base_x = self.base_x if base_x is None else float(base_x)
        phase = (t_local / gait.cycle) % 1.0 if gait.cycle > 0 else 0.0
        hip_z = gait.hip_height - gait.bob * math.cos(4.0 * math.pi * phase)
        q = np.zeros(6)
        for idx, side in enumerate(("left", "right")):
            x_rel, z_rel, _ = self._foot_relative(gait, side, phase)
            ankle = (base_x + x_rel, self.model.ankle_height + z_rel)
            q[3 * idx:3 * idx + 3] = self.model.leg_ik((base_x, hip_z), ankle)
        return q

    def _state(self, gait_name: str, t_local: float) -> dict:
        gait = self.gaits[gait_name]
        phase = (t_local / gait.cycle) % 1.0 if gait.cycle > 0 else 0.0
        hip_z = gait.hip_height - gait.bob * math.cos(4.0 * math.pi * phase)
        q = self._evaluate(gait_name, t_local)
        feet = []
        contacts = []
        for side in ("left", "right"):
            x_rel, z_rel, contact = self._foot_relative(gait, side, phase)
            feet.append((self.base_x + x_rel, self.model.ankle_height + z_rel))
            contacts.append(contact)
        return {
            "q": q, "phase": phase, "hip_z": hip_z, "lean": gait.lean,
            "contacts": contacts, "feet": feet,
        }

    def _start_transition(self, new_gait: str, t: float):
        old_velocity = self.gaits[self.gait_name].velocity
        new_velocity = self.gaits[new_gait].velocity
        end_time = t + self.transition_time
        predicted_x = self.base_x + 0.5 * (old_velocity + new_velocity) * self.transition_time
        q1 = self._evaluate(new_gait, self.transition_time, base_x=predicted_x)
        eps = 1e-4
        q1_next = self._evaluate(new_gait, self.transition_time + eps, base_x=predicted_x)
        dq1 = (q1_next - q1) / eps
        self.transition = {
            "t0": t, "t1": end_time,
            "q0": self.last_q.copy(), "dq0": self.last_dq.copy(),
            "q1": q1, "dq1": dq1,
            "old_velocity": old_velocity, "new_velocity": new_velocity,
            "target_gait": new_gait, "target_start": t,
        }
        self.gait_start = t

    def update(self, t: float, dt: float) -> dict:
        desired = self.gait_at(t)
        switch_event = False
        if desired != self.gait_name:
            if self.mode == "spline":
                self._start_transition(desired, t)
            else:
                self.transition = None
            self.gait_name = desired
            self.gait_start = t
            switch_event = True

        # 基座速度：过渡期内用 smoothstep 混合
        if self.transition is not None:
            u = (t - self.transition["t0"]) / max(self.transition["t1"] - self.transition["t0"], 1e-6)
            u = max(0.0, min(1.0, u))
            blend = 3.0 * u ** 2 - 2.0 * u ** 3
            v = self.transition["old_velocity"] + \
                (self.transition["new_velocity"] - self.transition["old_velocity"]) * blend
            if u >= 1.0:
                self.transition = None
        else:
            v = self.gaits[self.gait_name].velocity
        self.base_v = v
        self.base_x += v * dt

        state = self._state(self.gait_name, t - self.gait_start)
        q_target = state["q"]

        if self.transition is not None:
            tr = self.transition
            q = hermite_vector(t, tr["t0"], tr["q0"], tr["dq0"],
                               tr["t1"], tr["q1"], tr["dq1"])
            dq = hermite_vector_derivative(t, tr["t0"], tr["q0"], tr["dq0"],
                                           tr["t1"], tr["q1"], tr["dq1"])
            switching = True
            progress = (t - tr["t0"]) / max(tr["t1"] - tr["t0"], 1e-6)
        else:
            q = q_target
            # 稳态段用后向差分；切换段使用 Hermite 解析导数，保证切换首拍
            # 发布的速度等于上一拍速度而不是 0。
            dq = (q - self.last_q) / max(dt, 1e-6)
            switching = False
            progress = 0.0
        self.last_q = q.copy()
        self.last_dq = dq.copy()
        self.last_t = t

        return {
            "gait": self.gait_name,
            "phase": state["phase"],
            "q": q,
            "dq": dq,
            "q_target": q_target,
            "base_x": self.base_x,
            "base_v": self.base_v,
            "hip_z": state["hip_z"],
            "lean": state["lean"],
            "contacts": state["contacts"],
            "feet": state["feet"],
            "switching": switching,
            "switch_event": switch_event,
            "transition_progress": progress,
            "elapsed": t,
        }
