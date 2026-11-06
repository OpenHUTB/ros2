#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轨迹跟踪速度控制与坐标系工具。"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VelocityCommand:
    vx: float
    vy: float
    vz: float
    position_error: float


def body_frame_is_safe(value):
    """仅显式 false 允许发布世界 ENU 速度."""
    return value is not None and not bool(value)


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def body_offsets_to_world(start, yaw, offsets):
    """把 [前, 左, 上] 相对位置转换成绝对 ENU 航点."""
    start = np.asarray(start, dtype=np.float64)
    result = []
    cosine = math.cos(float(yaw))
    sine = math.sin(float(yaw))
    for item in offsets:
        offset = np.asarray(item, dtype=np.float64).reshape(3)
        forward = float(offset[0])
        left = float(offset[1])
        result.append(
            np.array(
                [
                    start[0] + cosine * forward - sine * left,
                    start[1] + sine * forward + cosine * left,
                    start[2] + float(offset[2]),
                ],
                dtype=np.float64,
            )
        )
    return result


def _clip_norm(vector, maximum):
    norm = float(np.linalg.norm(vector))
    if norm > float(maximum) > 0.0:
        return vector * (float(maximum) / norm)
    return vector


def _accel_limit(target, current, maximum, dt):
    delta = np.clip(
        target - current,
        -float(maximum) * max(dt, 0.0),
        float(maximum) * max(dt, 0.0),
    )
    return current + delta


def tracking_velocity(position, velocity, reference, params, dt):
    """前馈参考速度 + 位置反馈，并限制速度和加速度."""
    position = np.asarray(position, dtype=np.float64)
    velocity = np.asarray(velocity, dtype=np.float64)
    error = reference.position - position
    desired = reference.velocity + np.array(
        [
            float(params["position_gain_xy"]) * error[0]
            + float(params["velocity_gain_xy"])
            * (reference.velocity[0] - velocity[0]),
            float(params["position_gain_xy"]) * error[1]
            + float(params["velocity_gain_xy"])
            * (reference.velocity[1] - velocity[1]),
            float(params["position_gain_z"]) * error[2]
            + float(params["velocity_gain_z"])
            * (reference.velocity[2] - velocity[2]),
        ],
        dtype=np.float64,
    )

    horizontal = _clip_norm(
        desired[:2], float(params["max_horizontal_speed"])
    )
    desired[0:2] = horizontal
    desired[2] = float(
        np.clip(
            desired[2],
            -float(params["max_vertical_speed"]),
            float(params["max_vertical_speed"]),
        )
    )

    limited = np.array(
        [
            _accel_limit(
                desired[0], velocity[0],
                float(params["max_horizontal_accel"]), dt
            ),
            _accel_limit(
                desired[1], velocity[1],
                float(params["max_horizontal_accel"]), dt
            ),
            _accel_limit(
                desired[2], velocity[2],
                float(params["max_vertical_accel"]), dt
            ),
        ],
        dtype=np.float64,
    )
    horizontal = _clip_norm(
        limited[:2], float(params["max_horizontal_speed"])
    )
    limited[0:2] = horizontal
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
        position_error=float(np.linalg.norm(error)),
    )
