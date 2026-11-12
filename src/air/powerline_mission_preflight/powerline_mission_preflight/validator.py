"""Validate an inspection mission before it reaches a simulator or flight stack."""

from __future__ import annotations

import math
from collections import defaultdict
from typing import Any


REQUIRED_FIELDS = ("pole_index", "side", "x", "y", "z", "yaw_deg")


def _finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_mission(
    mission: dict[str, Any],
    *,
    min_altitude: float = 5.0,
    max_altitude: float = 120.0,
    max_leg_distance: float = 80.0,
) -> dict[str, Any]:
    """Return a structured preflight report without modifying the mission."""

    limits = (min_altitude, max_altitude, max_leg_distance)
    if not all(_finite_number(value) for value in limits):
        raise ValueError("preflight limits must be finite numbers")
    if min_altitude < 0 or max_altitude <= min_altitude or max_leg_distance <= 0:
        raise ValueError("preflight limits are inconsistent")

    errors: list[str] = []
    warnings: list[str] = []
    route = mission.get("route") if isinstance(mission, dict) else None
    if not isinstance(route, list) or not route:
        return {
            "ready": False,
            "errors": ["route must be a non-empty list"],
            "warnings": [],
            "metrics": {"viewpoint_count": 0, "pole_count": 0, "distance_m": 0.0},
        }

    valid_points: list[dict[str, Any]] = []
    sides_by_pole: dict[int, set[str]] = defaultdict(set)
    for position, point in enumerate(route, start=1):
        if not isinstance(point, dict):
            errors.append(f"route[{position}] must be an object")
            continue
        missing = [field for field in REQUIRED_FIELDS if field not in point]
        if missing:
            errors.append(f"route[{position}] missing fields: {', '.join(missing)}")
            continue
        if not isinstance(point["pole_index"], int) or isinstance(point["pole_index"], bool) or point["pole_index"] < 1:
            errors.append(f"route[{position}].pole_index must be a positive integer")
            continue
        if point["side"] not in ("left", "right"):
            errors.append(f"route[{position}].side must be left or right")
            continue
        if not all(_finite_number(point[field]) for field in ("x", "y", "z", "yaw_deg")):
            errors.append(f"route[{position}] coordinates and yaw must be finite numbers")
            continue

        altitude = float(point["z"])
        if altitude < min_altitude or altitude > max_altitude:
            errors.append(
                f"route[{position}].z={altitude:.2f} outside allowed range "
                f"[{min_altitude:.2f}, {max_altitude:.2f}]"
            )
        yaw = float(point["yaw_deg"])
        if yaw < -180.0 or yaw > 180.0:
            errors.append(f"route[{position}].yaw_deg must be within [-180, 180]")
        sides_by_pole[point["pole_index"]].add(point["side"])
        valid_points.append(point)

    distance = 0.0
    longest_leg = 0.0
    for position, (previous, current) in enumerate(zip(valid_points, valid_points[1:]), start=2):
        leg = math.sqrt(
            (float(current["x"]) - float(previous["x"])) ** 2
            + (float(current["y"]) - float(previous["y"])) ** 2
            + (float(current["z"]) - float(previous["z"])) ** 2
        )
        distance += leg
        longest_leg = max(longest_leg, leg)
        if leg > max_leg_distance:
            warnings.append(
                f"leg ending at route[{position}] is {leg:.2f} m, above {max_leg_distance:.2f} m"
            )

    for pole_index, sides in sorted(sides_by_pole.items()):
        missing_sides = {"left", "right"} - sides
        if missing_sides:
            warnings.append(f"pole {pole_index} missing views: {', '.join(sorted(missing_sides))}")

    return {
        "ready": not errors,
        "errors": errors,
        "warnings": warnings,
        "metrics": {
            "viewpoint_count": len(valid_points),
            "pole_count": len(sides_by_pole),
            "distance_m": round(distance, 3),
            "longest_leg_m": round(longest_leg, 3),
        },
    }
