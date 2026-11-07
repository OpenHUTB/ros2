#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务定义：航点序列、返航点与进度推进。"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

import numpy as np


@dataclass
class Mission:
    """一条“去程航点 → 返航”的任务。

    任务本身与链路无关：不管谁在给指令，机体到点即推进，因此三种控制策略可以
    用同一套进度口径比较。
    """

    home: np.ndarray
    waypoints: List[np.ndarray]
    hold_radius: float = 0.35

    index: int = 0
    finished: bool = False
    returned: bool = False
    home_reached: bool = False
    ever_left: bool = False
    history: List[int] = field(default_factory=list)

    def __post_init__(self):
        self.home = np.asarray(self.home, dtype=float)
        self.waypoints = [np.asarray(w, dtype=float) for w in self.waypoints]

    # ------------------------------------------------------------------
    def current_target(self) -> np.ndarray:
        """当前应飞往的点；去程走完后指向返航点。"""
        if self.index < len(self.waypoints):
            return self.waypoints[self.index]
        return self.home

    def advance(self, pos: np.ndarray) -> None:
        """到点推进；``home_reached`` 与航点是否走完无关。"""
        d_home = float(np.linalg.norm(np.asarray(pos) - self.home))
        if d_home > 2.0 * self.hold_radius:
            self.ever_left = True
        # 起点就在返航点，必须先离开过才算“返航到达”
        if self.ever_left and d_home <= self.hold_radius:
            self.home_reached = True
        if self.index < len(self.waypoints):
            if np.linalg.norm(np.asarray(pos) - self.waypoints[self.index]) <= self.hold_radius:
                self.index += 1
                self.history.append(self.index)
        elif not self.returned:
            if np.linalg.norm(np.asarray(pos) - self.home) <= self.hold_radius:
                self.returned = True
                self.finished = True

    def progress(self) -> float:
        """0~1：已完成的航段占比（返航占最后一段）。"""
        total = len(self.waypoints) + 1
        done = min(self.index, len(self.waypoints))
        if self.returned:
            done += 1
        return done / float(total)

    def nominal_path(self) -> np.ndarray:
        """名义航迹折线（起点由调用方补），用于偏差统计。"""
        pts = [self.home] + self.waypoints + [self.home]
        return np.array(pts, dtype=float)

    def reference_altitude(self) -> float:
        return float(np.mean([w[2] for w in self.waypoints])) if self.waypoints else 1.5
