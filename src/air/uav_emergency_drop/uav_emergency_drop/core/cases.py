#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Three built-in cases: village cluster, river detour, 8 m/s crosswind.

Single-line summary: case dataclasses bundle demand points, depot, wind,
drop height/speed and optional river band so CLI/tests/flyer share one
source of truth.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from .ballistics import BallisticsCfg, make_release_plan
from .planner import CargoCfg, DemandPoint


@dataclass(frozen=True)
class DropCase:
    """一个完整算例：需求点 + 环境 + 投放条件。"""

    name: str
    title: str
    points: Tuple[DemandPoint, ...]
    wind: Tuple[float, float] = (0.0, 0.0)
    drop_h: float = 30.0
    drop_speed: float = 8.0
    cargo: CargoCfg = CargoCfg()
    river: Optional[Tuple[float, float]] = None   # 禁飞带 (y_lo, y_hi)
    bridge_x: Optional[float] = None              # 桥梁通道 x
    ballistics: BallisticsCfg = BallisticsCfg()

    @property
    def plan(self):
        return make_release_plan(self.drop_h, self.drop_speed, self.wind,
                                 self.ballistics)

    def top_targets(self, k: int = 3) -> List[DemandPoint]:
        """闭环真飞用的前 k 个投放点（按权重降序）。"""
        return sorted(self.points, key=lambda p: (-p.weight, p.pid))[:k]


def _pts(*rows) -> Tuple[DemandPoint, ...]:
    return tuple(DemandPoint(*r) for r in rows)


CASE_AID_VILLAGE = DropCase(
    name="case_aid_village",
    title="Aid drop over a village cluster (no wind)",
    points=_pts(
        ("V1", 120.0, 60.0, 3, 5),
        ("V2", 210.0, 130.0, 4, 4),
        ("V3", 300.0, 90.0, 2, 5),
        ("V4", 150.0, 210.0, 3, 3),
        ("V5", 330.0, 200.0, 4, 4),
        ("V6", 260.0, 280.0, 2, 5),
        ("V7", 90.0, 300.0, 3, 2),
        ("V8", 400.0, 300.0, 2, 3),
    ),
    wind=(0.0, 0.0), drop_h=30.0, drop_speed=8.0,
)

CASE_AID_RIVER = DropCase(
    name="case_aid_river",
    title="Aid drop across a river band, straight line blocked",
    points=_pts(
        ("R1", 260.0, 250.0, 3, 5),
        ("R2", 340.0, 320.0, 4, 4),
        ("R3", 180.0, 300.0, 3, 4),
        ("R4", 400.0, 220.0, 2, 5),
        ("R5", 120.0, 60.0, 3, 3),
        ("R6", 300.0, 120.0, 2, 2),
    ),
    wind=(0.0, 0.0), drop_h=30.0, drop_speed=8.0,
    river=(150.0, 180.0), bridge_x=260.0,
)

CASE_AID_WIND = DropCase(
    name="case_aid_wind",
    title="Aid drop under 8 m/s easterly crosswind",
    points=_pts(
        ("W1", 140.0, 40.0, 3, 5),
        ("W2", 240.0, 120.0, 4, 4),
        ("W3", 340.0, 60.0, 3, 5),
        ("W4", 200.0, 220.0, 2, 4),
        ("W5", 360.0, 180.0, 3, 3),
        ("W6", 80.0, 180.0, 2, 3),
    ),
    wind=(8.0, 0.0), drop_h=20.0, drop_speed=6.0,
)

CASES = {c.name: c for c in
         (CASE_AID_VILLAGE, CASE_AID_RIVER, CASE_AID_WIND)}
