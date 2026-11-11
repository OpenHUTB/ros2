"""Minimal conflict-risk map from ego and VRU trajectory CSV files."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class EgoPoint:
    timestamp: float
    x: float
    y: float
    speed_mps: float


@dataclass(frozen=True)
class VruPoint:
    track_id: str
    vru_class: str
    timestamp: float
    x: float
    y: float


def load_ego_track(path: Path) -> list[EgoPoint]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"timestamp", "x", "y", "speed_mps"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("ego CSV needs timestamp,x,y,speed_mps")
        return [
            EgoPoint(
                float(row["timestamp"]),
                float(row["x"]),
                float(row["y"]),
                float(row["speed_mps"]),
            )
            for row in reader
        ]


def load_vru_tracks(path: Path) -> list[VruPoint]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"track_id", "class", "timestamp", "x", "y"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("VRU CSV needs track_id,class,timestamp,x,y")
        return [
            VruPoint(
                row["track_id"].strip(),
                row["class"].strip().lower(),
                float(row["timestamp"]),
                float(row["x"]),
                float(row["y"]),
            )
            for row in reader
        ]


SUPPORTED_CLASSES = frozenset({"pedestrian", "child", "cyclist"})


def _nearest_index(points: list[EgoPoint], timestamp: float,
                   tolerance: float) -> int | None:
    index = min(range(len(points)),
                key=lambda i: abs(points[i].timestamp - timestamp))
    if abs(points[index].timestamp - timestamp) > tolerance:
        return None
    return index


def _velocity_at(points, index: int) -> tuple[float, float]:
    """Estimate planar velocity from adjacent trajectory samples."""
    before = index - 1 if index > 0 else None
    after = index + 1 if index + 1 < len(points) else None
    if before is not None and after is not None:
        dt = points[after].timestamp - points[before].timestamp
        if dt > 0.0:
            return ((points[after].x - points[before].x) / dt,
                    (points[after].y - points[before].y) / dt)
    if before is not None:
        dt = points[index].timestamp - points[before].timestamp
        if dt > 0.0:
            return ((points[index].x - points[before].x) / dt,
                    (points[index].y - points[before].y) / dt)
    if after is not None:
        dt = points[after].timestamp - points[index].timestamp
        if dt > 0.0:
            return ((points[after].x - points[index].x) / dt,
                    (points[after].y - points[index].y) / dt)
    return 0.0, 0.0


def _time_to_collision(dx: float, dy: float,
                       relative_vx: float, relative_vy: float) -> float:
    """Relative-motion TTC; infinity means the pair is not closing."""
    closing_speed = -(dx * relative_vx + dy * relative_vy)
    if closing_speed <= 1e-9:
        return math.inf
    return (dx * dx + dy * dy) / closing_speed


def evaluate_conflicts(
    ego: Iterable[EgoPoint],
    vrus: Iterable[VruPoint],
    critical_distance_m: float = 3.0,
    caution_distance_m: float = 6.0,
    critical_ttc_s: float = 2.0,
    caution_ttc_s: float = 4.0,
    timestamp_tolerance: float = 0.05,
    allowed_classes: Iterable[str] = SUPPORTED_CLASSES,
) -> dict:
    ego_points = sorted(ego, key=lambda point: point.timestamp)
    vru_points = sorted(vrus, key=lambda point: (point.track_id, point.timestamp))
    if not ego_points:
        raise ValueError("ego track is empty")
    allowed = {name.strip().lower() for name in allowed_classes}
    tracks: dict[str, dict] = {}
    point_counts = {"critical": 0, "caution": 0, "safe": 0}
    grouped: dict[str, list[VruPoint]] = {}
    for point in vru_points:
        grouped.setdefault(point.track_id, []).append(point)

    skipped_points = 0
    skipped_tracks = 0
    invalid_class_points = 0
    order = {"safe": 0, "caution": 1, "critical": 2}
    for track_id, track_points in grouped.items():
        matched = False
        for point_index, vru in enumerate(track_points):
            if vru.vru_class not in allowed:
                invalid_class_points += 1
                continue
            ego_index = _nearest_index(
                ego_points, vru.timestamp, timestamp_tolerance)
            if ego_index is None:
                skipped_points += 1
                continue
            ego_point = ego_points[ego_index]
            dx, dy = vru.x - ego_point.x, vru.y - ego_point.y
            distance = math.hypot(dx, dy)
            ego_vx, ego_vy = _velocity_at(ego_points, ego_index)
            vru_vx, vru_vy = _velocity_at(track_points, point_index)
            ttc = _time_to_collision(
                dx, dy, vru_vx - ego_vx, vru_vy - ego_vy)
            if distance <= critical_distance_m or ttc <= critical_ttc_s:
                level = "critical"
            elif distance <= caution_distance_m or ttc <= caution_ttc_s:
                level = "caution"
            else:
                level = "safe"
            point_counts[level] += 1
            matched = True
            current = tracks.setdefault(track_id, {
                "track_id": track_id,
                "class": vru.vru_class,
                "min_distance_m": distance,
                "min_ttc_s": ttc,
                "risk": "safe",
            })
            current["min_distance_m"] = min(current["min_distance_m"], distance)
            current["min_ttc_s"] = min(current["min_ttc_s"], ttc)
            if order[level] > order[current["risk"]]:
                current["risk"] = level
        if not matched:
            skipped_tracks += 1

    result_tracks = sorted(tracks.values(), key=lambda track: track["track_id"])
    for track in result_tracks:
        if math.isinf(track["min_ttc_s"]):
            track["min_ttc_s"] = None
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
        "overall_risk": overall,
        "counts": counts,
        "point_counts": point_counts,
        "skipped_points": skipped_points,
        "skipped_tracks": skipped_tracks,
        "invalid_class_points": invalid_class_points,
        "warnings": warnings,
        "tracks": result_tracks,
    }
