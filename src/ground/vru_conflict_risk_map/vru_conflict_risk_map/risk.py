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


def _nearest(points: Iterable[EgoPoint], timestamp: float, tolerance: float) -> EgoPoint | None:
    candidates = [point for point in points if abs(point.timestamp - timestamp) <= tolerance]
    return min(candidates, key=lambda point: abs(point.timestamp - timestamp)) if candidates else None


def evaluate_conflicts(
    ego: Iterable[EgoPoint],
    vrus: Iterable[VruPoint],
    critical_distance_m: float = 3.0,
    caution_distance_m: float = 6.0,
    critical_ttc_s: float = 2.0,
    caution_ttc_s: float = 4.0,
    timestamp_tolerance: float = 0.05,
) -> dict:
    ego_points = list(ego)
    vru_points = list(vrus)
    if not ego_points:
        raise ValueError("ego track is empty")
    tracks: dict[str, dict] = {}
    counts = {"critical": 0, "caution": 0, "safe": 0}
    for vru in vru_points:
        if vru.vru_class not in {"pedestrian", "child", "cyclist"}:
            raise ValueError("unknown VRU class: " + vru.vru_class)
        ego_point = _nearest(ego_points, vru.timestamp, timestamp_tolerance)
        if ego_point is None:
            continue
        distance = math.hypot(vru.x - ego_point.x, vru.y - ego_point.y)
        ttc = distance / max(ego_point.speed_mps, 0.1)
        if distance <= critical_distance_m or ttc <= critical_ttc_s:
            level = "critical"
        elif distance <= caution_distance_m or ttc <= caution_ttc_s:
            level = "caution"
        else:
            level = "safe"
        counts[level] += 1
        current = tracks.setdefault(
            vru.track_id,
            {"track_id": vru.track_id, "class": vru.vru_class, "min_distance_m": distance, "min_ttc_s": ttc, "risk": "safe"},
        )
        current["min_distance_m"] = min(current["min_distance_m"], distance)
        current["min_ttc_s"] = min(current["min_ttc_s"], ttc)
        order = {"safe": 0, "caution": 1, "critical": 2}
        if order[level] > order[current["risk"]]:
            current["risk"] = level
    overall = "critical" if counts["critical"] else ("caution" if counts["caution"] else "safe")
    return {"overall_risk": overall, "counts": counts, "tracks": list(tracks.values())}
