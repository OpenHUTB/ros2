#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三维安全走廊与自适应高度选择。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CorridorChoice:
    """一次安全高度选择的结果."""

    target_z: float
    risk: float
    corridor_points: int
    candidate_count: int


def _horizontal_direction(position, goal):
    direction = np.asarray(goal[:2], dtype=np.float64) - np.asarray(
        position[:2], dtype=np.float64
    )
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        return np.array([1.0, 0.0], dtype=np.float64)
    return direction / norm


def _score_altitudes(points, position, goal, params):
    points = np.asarray(points, dtype=np.float64)
    position = np.asarray(position, dtype=np.float64)
    if points.size == 0:
        return None

    relative = points - position
    horizontal = relative[:, :2]
    distance = np.linalg.norm(horizontal, axis=1)
    direction = _horizontal_direction(position, goal)
    right = np.array([-direction[1], direction[0]], dtype=np.float64)
    along = horizontal.dot(direction)
    lateral = horizontal.dot(right)

    mask = (
        (distance >= float(params["lookahead_min"]))
        & (distance <= float(params["lookahead_max"]))
        & (np.abs(lateral) <= float(params["half_width"]))
    )
    if not np.any(mask):
        return None

    rel_z = relative[mask, 2]
    weights = 1.0 / (
        float(params["distance_weight"]) + np.maximum(along[mask], 0.0)
    )
    clearance = float(params["vertical_clearance"])
    offsets = np.arange(
        float(params["min_offset"]),
        float(params["max_offset"]) + 0.5 * float(params["offset_step"]),
        float(params["offset_step"]),
        dtype=np.float64,
    )
    candidates = position[2] + offsets
    scores = []
    risks = []
    for candidate in candidates:
        in_band = np.abs(rel_z - (candidate - position[2])) <= clearance
        risk = float(np.sum(weights[in_band]))
        score = (
            risk
            + float(params["goal_z_weight"]) * abs(candidate - float(goal[2]))
            + float(params["current_z_weight"]) * abs(candidate - position[2])
        )
        scores.append(score)
        risks.append(risk)

    best = int(np.argmin(scores))
    return CorridorChoice(
        target_z=float(candidates[best]),
        risk=float(risks[best]),
        corridor_points=int(np.count_nonzero(mask)),
        candidate_count=int(candidates.size),
    )


def choose_altitude(points, position, goal, params):
    """选择一条远离前方障碍且接近目标高度的走廊中心高度."""
    choice = _score_altitudes(points, position, goal, params)
    if choice is not None:
        return choice
    return CorridorChoice(
        target_z=float(goal[2]),
        risk=0.0,
        corridor_points=0,
        candidate_count=0,
    )


def distance_to_points(position, points):
    """返回当前位置到点云的最近距离；无点时返回 inf."""
    points = np.asarray(points, dtype=np.float64)
    if points.size == 0:
        return float("inf")
    delta = points - np.asarray(position, dtype=np.float64)
    return float(np.min(np.linalg.norm(delta, axis=1)))
