#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分级降级监督器：链路质量 → 指令来源 / 速度上限 / 目标点。

四态：

``NORMAL``    链路新鲜，遥控方掌舵（REMOTE），按遥控速度指令飞；
``DEGRADED``  链路质量下降，机载自主接管（ONBOARD），只沿已装订航点慢速推进，不再接收新航点；
``RETURN``    指令年龄超过失联阈值，放弃任务，爬回安全高度带后直线返航；
``LAND``      到返航点上方，低速下降，落地后停止。

切换带滞回：进入降级看命令年龄与滑窗投递率，恢复要求连续一段时间的“新鲜且高投递率”，
避免边界抖动把模式来回切换。状态机只接收 :class:`LinkStatus`，不读真实丢包标签。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Tuple

import numpy as np

from .link import LinkStatus
from .mission import Mission


class Mode(str, Enum):
    NORMAL = "NORMAL"
    DEGRADED = "DEGRADED"
    RETURN = "RETURN"
    LAND = "LAND"


@dataclass
class SupervisorParams:
    # 进入降级：命令年龄或投递率任一越限
    degrade_age: float = 0.45
    degrade_pdr: float = 0.55
    degrade_dwell: float = 0.20   # 退化条件需连续成立该时长才真正降级
    # 判定失联：命令年龄超过该值即放弃任务
    lost_age: float = 1.20
    # 恢复：命令新鲜且投递率高，持续该时长后退回 NORMAL
    recover_age: float = 0.50
    recover_pdr: float = 0.60
    recover_hold: float = 2.00
    # 速度上限（m/s）
    nominal_speed: float = 1.2
    degraded_speed: float = 0.8
    return_speed: float = 1.0
    land_speed: float = 0.35
    # 安全高度带（m）
    alt_min: float = 0.90
    alt_max: float = 2.40
    # 到点/回到返航点的判定半径
    arrive_radius: float = 0.35
    land_alt: float = 0.35


@dataclass
class Decision:
    """一拍决策结果。"""

    mode: Mode
    source: str                  # "remote" | "onboard"
    target: np.ndarray           # 机载自主时的目标点
    speed_cap: float
    reason: str


@dataclass
class SupervisorTrace:
    t: List[float] = field(default_factory=list)
    mode: List[str] = field(default_factory=list)
    age: List[float] = field(default_factory=list)
    pdr: List[float] = field(default_factory=list)
    reason: List[str] = field(default_factory=list)

    def add(self, t: float, mode: Mode, status: LinkStatus, reason: str) -> None:
        self.t.append(float(t))
        self.mode.append(mode.value)
        self.age.append(float(status.age))
        self.pdr.append(float(status.pdr))
        self.reason.append(reason)


class DegradationSupervisor:
    """链路驱动的分级降级状态机。"""

    def __init__(self, params: SupervisorParams | None = None):
        self.p = params or SupervisorParams()
        self.mode = Mode.NORMAL
        self.reason = "链路正常"
        self._recover_since: float | None = None
        self._degrade_since: float | None = None
        self._switches = 0
        self.trace = SupervisorTrace()

    # ------------------------------------------------------------------
    def _speed_cap(self, mode: Mode) -> float:
        return {
            Mode.NORMAL: self.p.nominal_speed,
            Mode.DEGRADED: self.p.degraded_speed,
            Mode.RETURN: self.p.return_speed,
            Mode.LAND: self.p.land_speed,
        }[mode]

    def _switch(self, mode: Mode, reason: str) -> None:
        self._degrade_since = None
        if mode != self.mode:
            self.mode = mode
            self.reason = reason
            self._switches += 1
            self._recover_since = None

    def _fresh_enough(self, s: LinkStatus) -> bool:
        return s.age <= self.p.recover_age and s.warm and s.pdr >= self.p.recover_pdr

    # ------------------------------------------------------------------
    def update(self, now: float, status: LinkStatus, pos: np.ndarray,
               mission: Mission) -> Decision:
        p = self.p
        pos = np.asarray(pos, dtype=float)

        if self.mode in (Mode.NORMAL, Mode.DEGRADED):
            if status.age > p.lost_age:
                self._switch(Mode.RETURN, f"命令年龄 {status.age:.2f}s 超过失联阈值 {p.lost_age:.2f}s")
            elif status.age > p.degrade_age or (status.warm and status.pdr < p.degrade_pdr):
                # 退化条件必须连续成立 degrade_dwell，避免单次丢包把模式打翻
                if self._degrade_since is None:
                    self._degrade_since = now
                elif now - self._degrade_since >= p.degrade_dwell:
                    self._switch(Mode.DEGRADED,
                                 f"链路退化（年龄 {status.age:.2f}s，投递率 {status.pdr:.2f}）")
            elif self.mode == Mode.DEGRADED:
                if self._fresh_enough(status):
                    if self._recover_since is None:
                        self._recover_since = now
                    elif now - self._recover_since >= p.recover_hold:
                        self._switch(Mode.NORMAL, "链路恢复且稳定")
                else:
                    self._recover_since = None
            else:
                self._degrade_since = None
                self.reason = "链路正常"

        elif self.mode == Mode.RETURN:
            # 返航是“已提交”的决策：链路恢复也不中途复飞，避免来回切任务
            if np.linalg.norm(pos - mission.home) <= p.arrive_radius:
                self._switch(Mode.LAND, "已到返航点上方，开始下降")
            elif not status.warm or status.age <= p.degrade_age:
                self.reason = "返航中（链路已恢复，但返航已提交）"

        elif self.mode == Mode.LAND:
            # 落地是终态：链路恢复也不自动复飞，交由地面人员复位
            if pos[2] <= p.land_alt:
                self.reason = "已落地"

        # 目标点：自主模式下只认已装订航点；返航模式直接指向 home
        if self.mode == Mode.DEGRADED:
            target = mission.current_target()
            target = np.array([target[0], target[1],
                               float(np.clip(target[2], p.alt_min, p.alt_max))])
        elif self.mode == Mode.RETURN:
            target = np.array([mission.home[0], mission.home[1], max(p.alt_min, mission.home[2])])
        elif self.mode == Mode.LAND:
            # 目标高度取“接地高度”，而不是 0：机体模型无碰撞体，压到 0 会穿地
            target = np.array([mission.home[0], mission.home[1], p.land_alt])
        else:
            target = mission.current_target()

        source = "remote" if self.mode == Mode.NORMAL else "onboard"
        decision = Decision(self.mode, source, target, self._speed_cap(self.mode), self.reason)
        self.trace.add(now, self.mode, status, self.reason)
        return decision

    # ------------------------------------------------------------------
    @property
    def switches(self) -> int:
        return self._switches
