#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""遥控端、机载跟踪律与两个对照策略。

对照策略刻意选成“工程上真会出现的两种做法”，而不是弱化过的靶子：

``hold_last``  指令超时后继续沿用上一条速度指令（很多地面站/遥控器的默认行为）；
``hover``      指令超时后置零悬停（``uav_mujoco_sim`` 里 ``cmd_timeout`` 的做法）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

import numpy as np

from .link import LinkStatus
from .mission import Mission
from .supervisor import Decision, DegradationSupervisor, Mode, SupervisorParams


# --------------------------------------------------------------------------- #
# 遥控端
# --------------------------------------------------------------------------- #
class RemotePilot:
    """遥控方：按“自己看到的位置”（下行遥测，可能延迟/丢包）朝当前航点打速度指令。"""

    def __init__(self, mission: Mission, cruise_speed: float = 2.0, gain: float = 1.2):
        self.mission = mission
        self.cruise_speed = float(cruise_speed)
        self.gain = float(gain)

    def command(self, belief_pos: np.ndarray) -> np.ndarray:
        target = self.mission.current_target()
        delta = np.asarray(target, dtype=float) - np.asarray(belief_pos, dtype=float)
        horiz = delta[:2]
        n = float(np.linalg.norm(horiz))
        if n < 1e-6:
            v_xy = np.zeros(2)
        else:
            v_xy = horiz / n * min(self.cruise_speed, self.gain * n)
        # 高度用小增益跟踪，避免遥控端把飞机压进地面
        v_z = float(np.clip(0.6 * delta[2], -0.8, 0.8))
        return np.array([v_xy[0], v_xy[1], v_z])


# --------------------------------------------------------------------------- #
# 机载跟踪律
# --------------------------------------------------------------------------- #
def follow(pos: np.ndarray, target: np.ndarray, speed_cap: float,
           alt_min: float = 0.9, alt_max: float = 2.4,
           gain: float = 1.2) -> np.ndarray:
    """位置误差 → 速度指令（限速 + 高度带限幅）。"""
    pos = np.asarray(pos, dtype=float)
    target = np.asarray(target, dtype=float).copy()
    target[2] = float(np.clip(target[2], alt_min, alt_max))
    delta = target - pos
    horiz = delta[:2]
    n = float(np.linalg.norm(horiz))
    v_xy = np.zeros(2) if n < 1e-6 else horiz / n * min(speed_cap, gain * n)
    v_z = float(np.clip(0.8 * delta[2], -0.9, 0.9))
    return np.array([v_xy[0], v_xy[1], v_z])


# --------------------------------------------------------------------------- #
# 策略
# --------------------------------------------------------------------------- #
@dataclass
class PolicyTrace:
    t: List[float] = field(default_factory=list)
    label: List[str] = field(default_factory=list)
    age: List[float] = field(default_factory=list)

    def add(self, t: float, label: str, age: float) -> None:
        self.t.append(float(t))
        self.label.append(label)
        self.age.append(float(age))


class HoldLastPolicy:
    """超时后继续使用最后一条速度指令。"""

    name = "hold_last"
    label = "沿用最后指令"

    def __init__(self, params: SupervisorParams | None = None):
        self.p = params or SupervisorParams()
        self.trace = PolicyTrace()
        self.switches = 0

    def decide(self, now: float, status: LinkStatus, pos, mission: Mission) -> Decision:
        label = "HOLD" if status.age > 0.5 else "LINK"
        self.trace.add(now, label, status.age)
        return Decision(Mode.NORMAL, "remote", mission.current_target(),
                        self.p.nominal_speed,
                        "沿用最后一条指令" if label == "HOLD" else "链路正常")


class HoverPolicy:
    """超时后置零悬停。"""

    name = "hover"
    label = "超时悬停"

    def __init__(self, params: SupervisorParams | None = None, timeout: float = 0.5):
        self.p = params or SupervisorParams()
        self.timeout = float(timeout)
        self.trace = PolicyTrace()
        self.switches = 0

    def decide(self, now: float, status: LinkStatus, pos, mission: Mission) -> Decision:
        stale = status.age > self.timeout
        label = "HOVER" if stale else "LINK"
        self.trace.add(now, label, status.age)
        return Decision(Mode.NORMAL, "zero" if stale else "remote",
                        mission.current_target(), 0.0 if stale else self.p.nominal_speed,
                        "超时置零悬停" if stale else "链路正常")


class SupervisedPolicy:
    """分级降级策略：包装 :class:`DegradationSupervisor`，提供统一接口。"""

    name = "supervised"
    label = "分级降级"

    def __init__(self, params: SupervisorParams | None = None):
        self.sv = DegradationSupervisor(params)
        self.trace = self.sv.trace
        self.switches = 0

    def decide(self, now: float, status: LinkStatus, pos, mission: Mission) -> Decision:
        d = self.sv.update(now, status, pos, mission)
        self.switches = self.sv.switches
        return d


def make_policy(name: str, params: SupervisorParams | None = None):
    table = {
        "hold_last": HoldLastPolicy,
        "hover": HoverPolicy,
        "supervised": SupervisedPolicy,
    }
    if name not in table:
        raise ValueError(f"未知策略 {name!r}，可选 {sorted(table)}")
    return table[name](params)
