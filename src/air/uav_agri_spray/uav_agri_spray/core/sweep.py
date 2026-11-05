#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Parameter sweep: speed x overlap grid, coverage-duration trade-off."""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import List

from .checker import assess
from .planner import SprayConfig, plan_field
from .scenarios import FieldCase, case_outline

TAG = "[sweep]"

#: 固定喷幅下的扫描维度
SWEEP_SPEEDS = (3.0, 4.0, 5.0)
SWEEP_OVERLAPS = (0.1, 0.2, 0.3)
#: 推荐组合的覆盖率门槛
MIN_COVERAGE = 0.98


@dataclass(frozen=True)
class SweepPoint:
    """One (speed, overlap) combination and its outcome."""

    speed: float
    overlap: float
    coverage_ratio: float
    overlap_ratio: float
    missed_area_m2: float
    total_length_m: float
    duration_s: float
    recommended: bool = False


def sweep_case(case: FieldCase, base_cfg: SprayConfig,
               speeds=SWEEP_SPEEDS, overlaps=SWEEP_OVERLAPS,
               min_coverage: float = MIN_COVERAGE) -> List[SweepPoint]:
    """Plan + assess every combination, then flag the recommended one.

    覆盖率只取决于重叠率（几何），时长 = 航程 / 速度；在覆盖率达标的组合
    里选时长最短者作为推荐参数。
    """
    points: List[SweepPoint] = []
    for ov in overlaps:
        cfg = SprayConfig(**{**base_cfg.__dict__, "overlap": ov})
        path = plan_field(case.subregions, case.obstacles, cfg)
        _grid, met = assess(path, case_outline(case), cfg)
        for sp in speeds:
            points.append(SweepPoint(
                speed=sp, overlap=ov,
                coverage_ratio=met.coverage_ratio,
                overlap_ratio=met.overlap_ratio,
                missed_area_m2=met.missed_area_m2,
                total_length_m=round(path.total_length, 1),
                duration_s=round(path.total_length / sp, 1)))
    best = recommend(points, min_coverage)
    points = [replace(p, recommended=(p.speed, p.overlap)
                                 == (best.speed, best.overlap))
              for p in points]
    return points


def recommend(points: List[SweepPoint],
              min_coverage: float = MIN_COVERAGE) -> SweepPoint:
    """Coverage-qualified shortest duration; fallback: best coverage first."""
    qualified = [p for p in points if p.coverage_ratio >= min_coverage]
    pool = qualified or points
    return min(pool, key=lambda p: (-p.coverage_ratio, p.duration_s,
                                    -p.overlap))
