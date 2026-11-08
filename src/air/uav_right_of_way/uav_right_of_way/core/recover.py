#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""让路恢复：脱离威胁后按余弦剖面平滑回到计划航路，恢复期间持续监视。

与参考实现 uav_deconflict ``plan.py`` 的 ``with_lateral_offset`` 同一模式：
偏移量沿恢复进程按 ``(1 + cos πs)/2`` 衰减——起点保持当前偏移、终点归零，
且两端变化率为零，因此不会出现航向/升降率阶跃去激励几何控制器的姿态
回路（参考实现实测：不平滑的折点偏移会激起增幅振荡、跟踪误差发散到
2×10^5 m）。

恢复不是一个"闭着眼执行完"的过程：调用方（``sim.py``）每个决策周期用
``should_abort`` 重新做威胁检测，一旦出现二次冲突立即中止恢复、重新进入
规则引擎——这就是"恢复期间持续监视防止二次冲突"。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

from .detect import DetectParams, Threat, predict_threat, AircraftState


@dataclass
class RecoveryProfile:
    """余弦剖面恢复：横向/高度偏移从当前值平滑衰减到 0，速度系数回到 1。

    Parameters
    ----------
    lateral0
        恢复开始时的横向偏移（m，沿计划航向左侧为正）。
    alt0
        恢复开始时的高度偏移（m，向上为正，通常为负值=降过高度）。
    speed_factor0
        恢复开始时的速度系数（<1 表示此前减过速）。
    duration
        恢复总时长（s）。
    """

    lateral0: float
    alt0: float
    speed_factor0: float = 1.0
    duration: float = 10.0

    def __post_init__(self) -> None:
        if self.duration <= 0.0:
            raise ValueError("恢复时长必须为正: %r" % (self,))

    def s(self, tau: float) -> float:
        """恢复进度 ∈ [0, 1]，``tau`` 为恢复开始后经过的秒数。"""
        return min(1.0, max(0.0, tau / self.duration))

    def _shape(self, s: float) -> float:
        return 0.5 * (1.0 + math.cos(math.pi * s))

    def lateral(self, s: float) -> float:
        return self.lateral0 * self._shape(s)

    def lateral_rate(self, s: float) -> float:
        """横向偏移的时间导数（m/s）。"""
        return -self.lateral0 * math.pi * math.sin(math.pi * s) \
            / (2.0 * self.duration)

    def alt(self, s: float) -> float:
        return self.alt0 * self._shape(s)

    def alt_rate(self, s: float) -> float:
        return -self.alt0 * math.pi * math.sin(math.pi * s) \
            / (2.0 * self.duration)

    def speed_factor(self, s: float) -> float:
        return 1.0 + (self.speed_factor0 - 1.0) * self._shape(s)


def should_abort(own: AircraftState, others: Sequence[AircraftState],
                 params: DetectParams = DetectParams()) -> Optional[Threat]:
    """恢复期间持续监视：出现新的预测威胁则返回该威胁（应中止恢复）。"""
    for other in others:
        if other.uid == own.uid:
            continue
        th = predict_threat(own, other, params)
        if th is not None:
            return th
    return None
