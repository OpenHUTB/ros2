#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Benchmark field cases: rect / obstacle / L-shape (two subregions)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from .geometry import CircleObstacle


@dataclass(frozen=True)
class FieldCase:
    """One benchmark field: convex subregions + circular obstacles."""

    name: str
    subregions: Tuple[np.ndarray, ...]
    obstacles: Tuple[CircleObstacle, ...] = ()


def _rect(x0: float, y0: float, x1: float, y1: float) -> np.ndarray:
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=float)


def _lshape_outline() -> np.ndarray:
    """L-shaped outline, counterclockwise (50 m east arm + 24 m north arm)."""
    return np.array([
        [0.0, 0.0], [50.0, 0.0], [50.0, 16.0],
        [24.0, 16.0], [24.0, 40.0], [0.0, 40.0],
    ], dtype=float)


CASES: List[FieldCase] = [
    FieldCase(
        name="case_field_rect",
        subregions=(_rect(0.0, 0.0, 50.0, 36.0),),
    ),
    FieldCase(
        name="case_field_obstacle",
        subregions=(_rect(0.0, 0.0, 56.0, 40.0),),
        obstacles=(CircleObstacle(28.0, 20.0, 0.8),   # 电线杆
                   CircleObstacle(12.0, 30.0, 1.5)),  # 树
    ),
    FieldCase(
        name="case_field_lshape",
        # L 形拆成两个凸子区：南臂 50×16 + 北臂 24×24
        subregions=(_rect(0.0, 0.0, 50.0, 16.0),
                    _rect(0.0, 16.0, 24.0, 40.0)),
    ),
]


def case_outline(case: FieldCase) -> np.ndarray:
    """True outline polygon for grid masking (handles concave L-shape)."""
    if case.name == "case_field_lshape":
        return _lshape_outline()
    return case.subregions[0]


def get_case(name: str) -> FieldCase:
    for c in CASES:
        if c.name == name:
            return c
    raise KeyError("unknown case %r (have %s)" %
                   (name, ", ".join(c.name for c in CASES)))
