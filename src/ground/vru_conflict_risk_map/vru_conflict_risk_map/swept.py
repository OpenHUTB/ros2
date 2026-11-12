"""Footprint-aware VRU conflict analysis using CPA and collision time."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from .risk import (
    SUPPORTED_CLASSES,
    EgoPoint,
    VruPoint,
    estimate_velocity,
    nearest_index,
)

DEFAULT_VRU_RADIUS_M = {
    "pedestrian": 0.30,
    "child": 0.25,
    "cyclist": 0.50,
}


@dataclass(frozen=True)
class SweptGeometry:
    time_to_closest_approach_s: float
    closest_approach_distance_m: float
    combined_radius_m: float
    min_clearance_m: float
    collision_ttc_s: float | None


def swept_geometry(dx: float, dy: float, relative_vx: float,
                   relative_vy: float, combined_radius_m: float) -> SweptGeometry:
    """Analyze constant relative motion against circular safety footprints."""
    speed_sq = relative_vx * relative_vx + relative_vy * relative_vy
    radial_dot_speed = dx * relative_vx + dy * relative_vy
    if speed_sq <= 1e-12:
        tcpa = 0.0
    else:
        tcpa = max(0.0, -radial_dot_speed / speed_sq)
    closest_x = dx + relative_vx * tcpa
    closest_y = dy + relative_vy * tcpa
    cpa_distance = math.hypot(closest_x, closest_y)
    radius = max(0.0, combined_radius_m)
    clearance = cpa_distance - radius

    center_distance_sq = dx * dx + dy * dy
    collision_ttc = None
    if center_distance_sq <= radius * radius:
        collision_ttc = 0.0
    elif speed_sq > 1e-12 and radial_dot_speed < 0.0:
        constant = center_distance_sq - radius * radius
        discriminant = radial_dot_speed * radial_dot_speed - speed_sq * constant
        if discriminant >= 0.0:
            candidate = (-radial_dot_speed - math.sqrt(discriminant)) / speed_sq
            if candidate >= 0.0:
                collision_ttc = candidate
    return SweptGeometry(tcpa, cpa_distance, radius, clearance, collision_ttc)


def evaluate_swept_conflicts(
    ego: Iterable[EgoPoint],
    vrus: Iterable[VruPoint],
    ego_radius_m: float = 1.0,
    vru_radii_m: dict[str, float] | None = None,
    caution_clearance_m: float = 1.0,
    critical_ttc_s: float = 2.0,
    horizon_s: float = 5.0,
    timestamp_tolerance: float = 0.05,
    allowed_classes: Iterable[str] = SUPPORTED_CLASSES,
) -> dict:
    """Evaluate future collision risk for circular ego/VRU footprints."""
    if ego_radius_m < 0.0:
        raise ValueError("ego_radius_m must be non-negative")
    if caution_clearance_m < 0.0:
        raise ValueError("caution_clearance_m must be non-negative")
    if horizon_s <= 0.0:
        raise ValueError("horizon_s must be positive")
    ego_points = sorted(ego, key=lambda point: point.timestamp)
    vru_points = sorted(vrus, key=lambda point: (point.track_id, point.timestamp))
    if not ego_points:
        raise ValueError("ego track is empty")
    allowed = {name.strip().lower() for name in allowed_classes}
    radii = dict(DEFAULT_VRU_RADIUS_M)
    radii.update({
        name.strip().lower(): float(radius)
        for name, radius in (vru_radii_m or {}).items()
    })
    if any(radius < 0.0 for radius in radii.values()):
        raise ValueError("VRU radii must be non-negative")
    ego_timestamps = [point.timestamp for point in ego_points]
    ego_velocities = [
        estimate_velocity(ego_points, index)
        for index in range(len(ego_points))
    ]
    grouped: dict[str, list[VruPoint]] = {}
    for point in vru_points:
        grouped.setdefault(point.track_id, []).append(point)

    tracks: dict[str, dict] = {}
    point_counts = {"critical": 0, "caution": 0, "safe": 0}
    skipped_points = 0
    skipped_tracks = 0
    invalid_class_points = 0
    order = {"safe": 0, "caution": 1, "critical": 2}

    for track_id, track_points in grouped.items():
        matched = False
        vru_velocities = [
            estimate_velocity(track_points, index)
            for index in range(len(track_points))
        ]
        for point_index, vru in enumerate(track_points):
            if vru.vru_class not in allowed:
                invalid_class_points += 1
                continue
            ego_index = nearest_index(
                ego_timestamps, vru.timestamp, timestamp_tolerance)
            if ego_index is None:
                skipped_points += 1
                continue
            ego_point = ego_points[ego_index]
            ego_vx, ego_vy = ego_velocities[ego_index]
            vru_vx, vru_vy = vru_velocities[point_index]
            radius = ego_radius_m + radii[vru.vru_class]
            geometry = swept_geometry(
                vru.x - ego_point.x,
                vru.y - ego_point.y,
                vru_vx - ego_vx,
                vru_vy - ego_vy,
                radius,
            )
            horizon_time = min(geometry.time_to_closest_approach_s, horizon_s)
            horizon_x = (
                vru.x - ego_point.x
                + (vru_vx - ego_vx) * horizon_time
            )
            horizon_y = (
                vru.y - ego_point.y
                + (vru_vy - ego_vy) * horizon_time
            )
            horizon_clearance = math.hypot(horizon_x, horizon_y) - radius
            collision_in_horizon = (
                geometry.collision_ttc_s is not None
                and geometry.collision_ttc_s <= horizon_s
            )
            if (geometry.collision_ttc_s is not None
                    and geometry.collision_ttc_s <= critical_ttc_s):
                level = "critical"
            elif collision_in_horizon:
                level = "caution"
            elif (geometry.time_to_closest_approach_s <= horizon_s
                  and horizon_clearance <= caution_clearance_m):
                level = "caution"
            else:
                level = "safe"
            point_counts[level] += 1
            matched = True

            current = tracks.setdefault(track_id, {
                "track_id": track_id,
                "class": vru.vru_class,
                "combined_radius_m": radius,
                "min_clearance_m": horizon_clearance,
                "min_clearance_timestamp_s": vru.timestamp,
                "closest_approach_distance_m": geometry.closest_approach_distance_m,
                "closest_approach_time_s": geometry.time_to_closest_approach_s,
                "collision_ttc_s": geometry.collision_ttc_s,
                "collision_ttc_timestamp_s": (
                    vru.timestamp
                    if geometry.collision_ttc_s is not None else None
                ),
                "collision_within_horizon": collision_in_horizon,
                "risk": "safe",
                "risk_timestamp_s": vru.timestamp,
            })
            if horizon_clearance < current["min_clearance_m"]:
                current["min_clearance_m"] = horizon_clearance
                current["min_clearance_timestamp_s"] = vru.timestamp
                current["closest_approach_distance_m"] = (
                    geometry.closest_approach_distance_m
                )
                current["closest_approach_time_s"] = (
                    geometry.time_to_closest_approach_s
                )
            if (geometry.collision_ttc_s is not None
                    and (current["collision_ttc_s"] is None
                         or geometry.collision_ttc_s
                         < current["collision_ttc_s"])):
                current["collision_ttc_s"] = geometry.collision_ttc_s
                current["collision_ttc_timestamp_s"] = vru.timestamp
            if collision_in_horizon:
                current["collision_within_horizon"] = True
            if order[level] > order[current["risk"]]:
                current["risk"] = level
                current["risk_timestamp_s"] = vru.timestamp
        if not matched:
            skipped_tracks += 1

    result_tracks = sorted(tracks.values(), key=lambda track: track["track_id"])
    counts = {"critical": 0, "caution": 0, "safe": 0}
    for track in result_tracks:
        counts[track["risk"]] += 1
    if not result_tracks:
        overall = "no_data"
    elif counts["critical"]:
        overall = "critical"
    elif counts["caution"]:
        overall = "caution"
    else:
        overall = "safe"
    warnings = []
    if skipped_points:
        warnings.append(
            f"{skipped_points} VRU point(s) had no ego sample within "
            f"{timestamp_tolerance:.3f} s")
    if invalid_class_points:
        warnings.append(
            f"{invalid_class_points} VRU point(s) used an unsupported class")
    return {
        "analysis": "swept",
        "overall_risk": overall,
        "ego_radius_m": ego_radius_m,
        "vru_radii_m": dict(sorted(radii.items())),
        "horizon_s": horizon_s,
        "counts": counts,
        "point_counts": point_counts,
        "skipped_points": skipped_points,
        "skipped_tracks": skipped_tracks,
        "invalid_class_points": invalid_class_points,
        "warnings": warnings,
        "tracks": result_tracks,
    }
