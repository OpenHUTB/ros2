"""简化的前向雷达射线与环道间隙特征。"""

from __future__ import annotations

import math
from collections.abc import Iterable


def raycast_ranges(
    origin: tuple[float, float],
    yaw: float,
    circles: Iterable[tuple[float, float, float]],
    rays: int = 25,
    max_range: float = 30.0,
) -> list[float]:
    """Return nearest ray-circle intersections over a forward 180-degree field."""
    if rays < 2 or max_range <= 0:
        raise ValueError("rays must be >= 2 and max_range must be positive")
    objects = list(circles)
    result = []
    for index in range(rays):
        angle = yaw - math.pi / 2 + math.pi * index / (rays - 1)
        dx, dy = math.cos(angle), math.sin(angle)
        nearest = max_range
        for x, y, radius in objects:
            ox, oy = origin[0] - x, origin[1] - y
            b = 2.0 * (ox * dx + oy * dy)
            c = ox * ox + oy * oy - radius * radius
            discriminant = b * b - 4.0 * c
            if discriminant < 0:
                continue
            root = math.sqrt(discriminant)
            hits = [t for t in ((-b - root) / 2.0, (-b + root) / 2.0) if 0.0 <= t <= nearest]
            if hits:
                nearest = min(hits)
        result.append(nearest)
    return result


def closest_gap(
    ego: tuple[float, float, float, float],
    traffic: Iterable[tuple[float, float, float, float]],
    max_range: float = 30.0,
) -> dict[str, float]:
    """Estimate closest forward gap, closing speed, lateral offset and TTC."""
    x, y, yaw, speed = ego
    fx, fy = math.cos(yaw), math.sin(yaw)
    lx, ly = -fy, fx
    best = None
    for tx, ty, tvx, tvy in traffic:
        rx, ry = tx - x, ty - y
        forward = rx * fx + ry * fy
        lateral = rx * lx + ry * ly
        if forward <= 0.0 or math.hypot(forward, lateral) > max_range:
            continue
        closing = speed - (tvx * fx + tvy * fy)
        ttc = forward / closing if closing > 1e-6 else max_range
        candidate = (forward, lateral, closing, min(ttc, max_range))
        if best is None or candidate[0] < best[0]:
            best = candidate
    if best is None:
        return {"gap_m": max_range, "lateral_m": 0.0, "closing_mps": 0.0, "ttc_s": max_range}
    return dict(zip(("gap_m", "lateral_m", "closing_mps", "ttc_s"), best))
