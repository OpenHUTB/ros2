#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""面向 OpenHUTB 桥接的速度控制。"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VelocityCommand:
    """世界 ENU 速度指令及诊断量."""

    vx: float
    vy: float
    vz: float
    nearest_distance: float
    speed_scale: float


def _limit_norm(vector, maximum):
    norm = float(np.linalg.norm(vector))
    if norm > maximum > 0.0:
        return vector * (maximum / norm)
    return vector


def bearing_error_deg(position, goal, heading_rad):
    """返回目标方向相对机头航向的最小夹角（度）."""
    dx = float(goal[0]) - float(position[0])
    dy = float(goal[1]) - float(position[1])
    target_heading = math.atan2(dy, dx)
    delta = (target_heading - float(heading_rad) + math.pi) % (2.0 * math.pi) - math.pi
    return abs(math.degrees(delta))


def _accel_limit(target, current, maximum, dt):
    limit = maximum * max(dt, 0.0)
    delta = np.clip(target - current, -limit, limit)
    return current + delta


def _forward_points(
    points,
    position,
    goal,
    lateral_width,
    target_z,
    vertical_influence,
    vertical_influence_below,
):
    points = np.asarray(points, dtype=np.float64)
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float64)

    relative = points - np.asarray(position, dtype=np.float64)
    direction = np.asarray(goal[:2], dtype=np.float64) - position[:2]
    direction_norm = float(np.linalg.norm(direction))
    if direction_norm < 1e-9:
        return np.empty((0, 3), dtype=np.float64)
    direction /= direction_norm
    right = np.array([-direction[1], direction[0]], dtype=np.float64)
    along = relative[:, :2].dot(direction)
    lateral = relative[:, :2].dot(right)
    mask = (along > 0.0) & (along <= float(lateral_width[1]))
    mask &= np.abs(lateral) <= float(lateral_width[0])
    relative_z = points[:, 2] - float(target_z)
    mask &= relative_z >= -float(vertical_influence_below)
    mask &= relative_z <= float(vertical_influence)
    return relative[mask]


def compute_velocity(
    position,
    velocity,
    goal,
    target_z,
    points,
    params,
    dt,
    depth_speed_scale=1.0,
):
    """根据目标点、安全高度和点云生成世界系 ENU 速度."""
    position = np.asarray(position, dtype=np.float64)
    velocity = np.asarray(velocity, dtype=np.float64)
    goal = np.asarray(goal, dtype=np.float64)
    to_goal = goal - position
    horizontal_to_goal = to_goal[:2]
    horizontal_distance = float(np.linalg.norm(horizontal_to_goal))

    if (
        horizontal_distance <= float(params["goal_tolerance_xy"])
        and abs(goal[2] - position[2]) <= float(params["goal_tolerance_z"])
    ):
        return VelocityCommand(0.0, 0.0, 0.0, float("inf"), 0.0)

    direction = (
        horizontal_to_goal / horizontal_distance
        if horizontal_distance > 1e-9
        else np.array([1.0, 0.0], dtype=np.float64)
    )
    forward = _forward_points(
        points,
        position,
        goal,
        (
            float(params["forward_corridor_half_width"]),
            float(params["slow_distance"]),
        ),
        target_z,
        float(params["vertical_influence"]),
        float(params["vertical_influence_below"]),
    )
    if forward.size:
        forward_distance = float(np.min(np.linalg.norm(forward, axis=1)))
    else:
        forward_distance = float("inf")

    stop = float(params["stop_distance"])
    slow = max(float(params["slow_distance"]), stop + 1e-6)
    if not np.isfinite(forward_distance) or forward_distance >= slow:
        lidar_scale = 1.0
    else:
        lidar_scale = float(
            np.clip((forward_distance - stop) / (slow - stop), 0.0, 1.0)
        )
    depth_scale = float(depth_speed_scale)
    speed_scale = min(lidar_scale, depth_scale)

    desired_h = direction * float(params["max_horizontal_speed"]) * lidar_scale

    # 近障碍的横向斥力只在目标高度附近生效；高度选择负责垂直绕行。
    points = np.asarray(points, dtype=np.float64)
    if points.size:
        relative = points - position
        horizontal = relative[:, :2]
        distance = np.linalg.norm(horizontal, axis=1)
        relative_z = points[:, 2] - float(target_z)
        near = (
            (distance > 1e-6)
            & (distance <= float(params["repulsion_radius"]))
            & (relative_z >= -float(params["vertical_influence_below"]))
            & (relative_z <= float(params["vertical_influence"]))
        )
        if np.any(near):
            weights = (
                float(params["repulsion_radius"]) - distance[near]
            ) / float(params["repulsion_radius"])
            repel = np.sum(
                (-horizontal[near] / distance[near, None])
                * weights[:, None],
                axis=0,
            )
            desired_h = desired_h + float(params["repulsion_gain"]) * repel
    desired_h = desired_h * depth_scale
    desired_h = _limit_norm(desired_h, float(params["max_horizontal_speed"]))

    desired_v = np.clip(
        float(params["vertical_gain"]) * (float(target_z) - position[2]),
        -float(params["max_vertical_speed"]),
        float(params["max_vertical_speed"]),
    )
    target_velocity = np.array(
        [desired_h[0], desired_h[1], float(desired_v)], dtype=np.float64
    )
    current = np.array(
        [velocity[0], velocity[1], velocity[2]], dtype=np.float64
    )
    limited = np.array(
        [
            _accel_limit(
                target_velocity[0], current[0],
                float(params["max_horizontal_accel"]), dt
            ),
            _accel_limit(
                target_velocity[1], current[1],
                float(params["max_horizontal_accel"]), dt
            ),
            _accel_limit(
                target_velocity[2], current[2],
                float(params["max_vertical_accel"]), dt
            ),
        ],
        dtype=np.float64,
    )
    horizontal = _limit_norm(
        limited[:2], float(params["max_horizontal_speed"])
    )
    limited[:2] = horizontal
    limited[2] = float(
        np.clip(
            limited[2],
            -float(params["max_vertical_speed"]),
            float(params["max_vertical_speed"]),
        )
    )
    return VelocityCommand(
        vx=float(limited[0]),
        vy=float(limited[1]),
        vz=float(limited[2]),
        nearest_distance=forward_distance,
        speed_scale=speed_scale,
    )
