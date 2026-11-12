# Copyright 2026 xxbzy727749
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Generate alternating inspection viewpoints around utility poles."""

from __future__ import annotations

import math
from typing import Iterable


def _positive(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a positive finite number")
    return value


def _pole(point: Iterable[float]) -> tuple[float, float, float]:
    try:
        values = tuple(float(value) for value in point)
    except (TypeError, ValueError) as exc:
        raise ValueError("each pole must contain finite x, y, and height values") from exc
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("each pole must contain finite x, y, and height values")
    if values[2] <= 0:
        raise ValueError("pole height must be positive")
    return values


def build_inspection_route(
    poles: Iterable[Iterable[float]],
    *,
    standoff: float = 8.0,
    height_margin: float = 2.0,
) -> list[dict[str, float | int | str]]:
    """
    Return alternating left/right viewpoints facing each pole.

    Pole coordinates use a local ENU frame. Two viewpoints are emitted for each
    pole so that both sides of the conductor corridor are inspected.
    """
    points = [_pole(point) for point in poles]
    if len(points) < 2:
        raise ValueError("at least two poles are required")
    standoff = _positive("standoff", standoff)
    height_margin = _positive("height_margin", height_margin)

    for previous, current in zip(points, points[1:]):
        if previous[:2] == current[:2]:
            raise ValueError("adjacent poles must not share the same horizontal position")

    route: list[dict[str, float | int | str]] = []
    for index, (x, y, height) in enumerate(points):
        previous = points[max(0, index - 1)]
        following = points[min(len(points) - 1, index + 1)]
        dx = following[0] - previous[0]
        dy = following[1] - previous[1]
        segment_length = math.hypot(dx, dy)
        if segment_length == 0:
            raise ValueError(
                "local pole direction must not reverse to the same horizontal position"
            )
        normal_x = -dy / segment_length
        normal_y = dx / segment_length

        side_order = ("left", "right") if index % 2 == 0 else ("right", "left")
        for side in side_order:
            sign = 1.0 if side == "left" else -1.0
            view_x = x + sign * standoff * normal_x
            view_y = y + sign * standoff * normal_y
            route.append(
                {
                    "pole_index": index + 1,
                    "side": side,
                    "x": view_x,
                    "y": view_y,
                    "z": height + height_margin,
                    "yaw_deg": math.degrees(math.atan2(y - view_y, x - view_x)),
                }
            )
    return route


def estimate_route(
    route: Iterable[dict[str, float | int | str]], speed: float
) -> dict[str, float | int]:
    """Return viewpoint count, route distance, and ideal travel time."""
    speed = _positive("speed", speed)
    points = list(route)
    distance = 0.0
    for previous, current in zip(points, points[1:]):
        distance += math.sqrt(
            (float(current["x"]) - float(previous["x"])) ** 2
            + (float(current["y"]) - float(previous["y"])) ** 2
            + (float(current["z"]) - float(previous["z"])) ** 2
        )
    return {
        "viewpoint_count": len(points),
        "distance_m": round(distance, 3),
        "ideal_time_s": round(distance / speed, 3),
    }
