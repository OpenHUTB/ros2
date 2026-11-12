"""Pure helpers for deterministic planar sonar simulation and map fusion."""

from __future__ import annotations

import math
from typing import Iterable, Sequence, Tuple

import numpy as np


# 注意：这里是**运行时求值**的别名赋值，不是注解，``from __future__ import annotations``
# 管不到它。内置泛型 tuple[...] 在 Python 3.8（课程虚拟机的版本）会抛
# "TypeError: 'type' object is not subscriptable"，所以用 typing.Tuple。
Obstacle = Tuple[float, float, float]
SonarReturn = Tuple[float, float, bool]
DEFAULT_MAP_ORIGIN = (0.0, 0.0)
DEFAULT_INITIAL_POSE = (10.0, 10.0, 0.0)
DEFAULT_OBSTACLES: tuple[Obstacle, ...] = (
    (5.0, 5.0, 1.0),
    (12.0, 15.0, 1.2),
    (15.0, 8.0, 1.0),
)


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def validate_obstacles(obstacles: Iterable[Sequence[float]]) -> tuple[Obstacle, ...]:
    """Normalize circles represented as ``(x, y, radius)`` triples."""
    normalized = []
    for index, obstacle in enumerate(obstacles):
        if len(obstacle) != 3:
            raise ValueError(f"obstacle {index} must contain x, y, and radius")
        x, y, radius = (_finite(value, f"obstacle {index}") for value in obstacle)
        if radius <= 0:
            raise ValueError(f"obstacle {index} radius must be positive")
        normalized.append((x, y, radius))
    return tuple(normalized)


def raycast_scan(
    x: float,
    y: float,
    yaw: float,
    obstacles: Iterable[Sequence[float]],
    beam_count: int = 60,
    field_of_view: float = math.tau,
    max_range: float = 10.0,
) -> tuple[SonarReturn, ...]:
    """Return evenly spaced simulated sonar beams and nearest circle hits."""
    x = _finite(x, "x")
    y = _finite(y, "y")
    yaw = _finite(yaw, "yaw")
    field_of_view = _finite(field_of_view, "field_of_view")
    max_range = _finite(max_range, "max_range")
    if not isinstance(beam_count, int) or isinstance(beam_count, bool) or beam_count < 1:
        raise ValueError("beam_count must be a positive integer")
    if not 0 < field_of_view <= math.tau:
        raise ValueError("field_of_view must be in (0, 2*pi]")
    if max_range <= 0:
        raise ValueError("max_range must be positive")

    circles = validate_obstacles(obstacles)
    scan = []
    for beam_index in range(beam_count):
        offset = 0.0 if beam_count == 1 else -field_of_view / 2 + field_of_view * beam_index / (beam_count - 1)
        angle = yaw + offset
        dx = math.cos(angle)
        dy = math.sin(angle)
        nearest = max_range
        hit = False

        for center_x, center_y, radius in circles:
            fx = x - center_x
            fy = y - center_y
            projection = fx * dx + fy * dy
            discriminant = projection * projection - (fx * fx + fy * fy - radius * radius)
            if discriminant < 0:
                continue
            root = math.sqrt(discriminant)
            distance = -projection - root
            if distance < 0:
                distance = -projection + root
            if 0 <= distance <= nearest:
                nearest = distance
                hit = True

        scan.append((angle, nearest, hit))
    return tuple(scan)


def integrate_scan(
    grid: np.ndarray,
    map_origin: tuple[float, float],
    resolution: float,
    x: float,
    y: float,
    scan: Iterable[SonarReturn],
) -> np.ndarray:
    """Fuse beam free-space and obstacle endpoints into an occupancy grid."""
    resolution = _finite(resolution, "resolution")
    x = _finite(x, "x")
    y = _finite(y, "y")
    origin_x, origin_y = (_finite(value, "map_origin") for value in map_origin)
    if resolution <= 0:
        raise ValueError("resolution must be positive")
    if grid.ndim != 2 or grid.size == 0:
        raise ValueError("grid must be a non-empty two-dimensional array")
    if np.any((grid < -1) | (grid > 100)):
        raise ValueError("grid values must be in [-1, 100]")

    result = np.asarray(grid, dtype=np.int8).copy()
    height, width = result.shape

    def cell(world_x: float, world_y: float) -> tuple[int, int] | None:
        column = math.floor((world_x - origin_x) / resolution)
        row = math.floor((world_y - origin_y) / resolution)
        if 0 <= row < height and 0 <= column < width:
            return row, column
        return None

    for angle, distance, hit in scan:
        angle = _finite(angle, "scan angle")
        distance = _finite(distance, "scan distance")
        if distance < 0:
            raise ValueError("scan distance must be non-negative")
        steps = max(1, math.ceil(distance / (resolution * 0.5)))
        for step in range(steps + 1):
            fraction = min(1.0, step / steps)
            point_x = x + math.cos(angle) * distance * fraction
            point_y = y + math.sin(angle) * distance * fraction
            current = cell(point_x, point_y)
            if current is None:
                break
            row, column = current
            is_endpoint = hit and step == steps
            if is_endpoint:
                result[row, column] = 100
            elif result[row, column] != 100:
                result[row, column] = 0
    return result
