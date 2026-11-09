"""Geometric baseline: waypoint pursuit + lidar potential field.

The baseline is intentionally simple and has no learned parameters.  It serves
two purposes:

1. It keeps ``roslaunch ... mode:=auto`` usable before any model is trained,
   so the ROS interface can be verified independently of the RL stack.
2. It is a reproducible reference for the DRL policy: if the trained policy
   cannot beat pure pursuit + repulsion, the learned result is not meaningful.

中文说明：纯几何基线（航点追踪 + 激光势场避障）。不依赖任何模型权重，既能在
训练前验证 ROS 接口，也作为强化学习策略的可复现对照。

Developer: 汤湘粤 (学号: 2535030067)
"""

from __future__ import annotations

import math
from typing import Any, Mapping, Sequence

import numpy as np

from common import (
    front_min_distance,
    observation_layout,
    sector_angles,
    wrap_angle,
)


def baseline_action(observation: Sequence[float],
                    cfg: Mapping[str, Any]) -> np.ndarray:
    """Return a normalized ``[-1, 1]`` action for the shared observation."""
    observation = np.asarray(observation, dtype=np.float64).reshape(-1)
    sectors = int(cfg["observation"]["sectors"])
    layout = observation_layout(sectors)
    goal_norm = max(float(cfg["goal"]["goal_norm"]), 1e-6)
    max_vel_z = max(float(cfg["limits"]["max_vel_z"]), 1e-6)

    goal_body = observation[layout["goal_body"]] * goal_norm
    histogram = observation[layout["lidar"]]

    distance = float(np.linalg.norm(goal_body))
    heading = math.atan2(float(goal_body[1]), float(goal_body[0]))
    alignment = max(0.0, math.cos(heading))
    speed = min(1.0, distance / 2.0)

    # 航点追踪：目标在身后时先原地转向，避免横向漂移。
    vx = speed * alignment
    vy = 0.5 * speed * math.sin(heading)
    vz = float(np.clip(goal_body[2] * 0.8 / max_vel_z, -1.0, 1.0))
    yaw_rate = float(np.clip(1.5 * heading, -1.0, 1.0))

    # 势场排斥：只在安全半径内、且只对前向半平面的障碍生效。
    # 直方图值 free = 最近距离 / 量程，因此 clearance = free * 量程。
    lidar_range = max(float(cfg["observation"]["lidar_range"]), 1e-6)
    avoid_distance = max(float(cfg["safety"].get("avoid_distance", 2.0)), 1e-6)
    gain = float(cfg["safety"].get("avoid_gain", 1.5))
    repulsive_x = 0.0
    repulsive_y = 0.0
    for index, free_space in enumerate(np.asarray(histogram, dtype=np.float64)):
        clearance = float(np.clip(free_space, 0.0, 1.0)) * lidar_range
        if clearance >= avoid_distance:
            continue
        angle = wrap_angle(float(sector_angles(sectors)[index]))
        if abs(angle) > math.pi / 2.0:
            continue
        # 障碍越近权重越大；排斥方向与障碍方位相反。
        weight = (1.0 - clearance / avoid_distance) ** 2
        repulsive_x -= weight * math.cos(angle)
        repulsive_y -= weight * math.sin(angle)

    vx = float(np.clip(vx + gain * repulsive_x, -1.0, 1.0))
    vy = float(np.clip(vy + gain * repulsive_y, -1.0, 1.0))

    # 正前方贴脸障碍：刹停平移速度，保留转向以便脱离。
    stop_distance = float(cfg["safety"]["stop_distance"])
    if front_min_distance(histogram, cfg) < stop_distance:
        vx = 0.0
        vy = 0.0
    if abs(repulsive_y) > 1e-6:
        yaw_rate = float(np.clip(yaw_rate + 1.2 * repulsive_y, -1.0, 1.0))

    return np.array([vx, vy, vz, yaw_rate], dtype=np.float32)
