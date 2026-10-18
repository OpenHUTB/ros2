#!/usr/bin/env python3
"""Track loading and evaluation for pedestrian/cyclist digital twins.

The module has no CARLA dependency at import time, so its metrics can be tested
on a laptop or in CI without a simulator.  A CARLA scene can feed it tracks
produced by a roadside camera/LiDAR pipeline, a CSV annotation file, or a
simulated ground-truth export.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


VALID_VRU_CLASSES = ("pedestrian", "child", "cyclist")
REQUIRED_COLUMNS = ("track_id", "class", "timestamp", "x", "y")


@dataclass(frozen=True)
class VruTrack:
    track_id: str
    vru_class: str
    timestamp: float
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0
    speed: float = 0.0
    confidence: float = 1.0


@dataclass(frozen=True)
class VruMetrics:
    gt_points: int
    pred_points: int
    matched_points: int
    false_negatives: int
    false_positives: int
    id_switches: int
    mota: float
    ade_m: Optional[float]
    fde_m: Optional[float]
    count_mae: float

    def as_dict(self) -> Dict[str, object]:
        return asdict(self)


def _finite(value: Optional[str], field: str, line: int, default: Optional[float] = None) -> float:
    if value is None or not str(value).strip():
        if default is not None:
            return default
        raise ValueError("line {}: {} is empty".format(line, field))
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError("line {}: {} is not a number: {!r}".format(line, field, value))
    if not math.isfinite(number):
        raise ValueError("line {}: {} is not finite".format(line, field))
    return number


def load_vru_tracks(path: Path) -> List[VruTrack]:
    """Load and validate a VRU track CSV.

    Required headers: track_id,class,timestamp,x,y
    Optional headers: z,yaw,speed,confidence
    """

    if not path.is_file():
        raise FileNotFoundError("VRU track CSV not found: {}".format(path))
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        columns = set(reader.fieldnames or [])
        missing = [name for name in REQUIRED_COLUMNS if name not in columns]
        if missing:
            raise ValueError("VRU CSV is missing columns: {}".format(", ".join(missing)))

        tracks: List[VruTrack] = []
        for line, row in enumerate(reader, start=2):
            track_id = (row.get("track_id") or "").strip()
            if not track_id:
                raise ValueError("line {}: track_id is empty".format(line))
            vru_class = (row.get("class") or "").strip().lower()
            if vru_class not in VALID_VRU_CLASSES:
                raise ValueError(
                    "line {}: class must be one of {}, got {!r}".format(
                        line, ", ".join(VALID_VRU_CLASSES), vru_class
                    )
                )
            confidence = _finite(row.get("confidence"), "confidence", line, default=1.0)
            if confidence < 0.0 or confidence > 1.0:
                raise ValueError("line {}: confidence must be in [0, 1]".format(line))
            tracks.append(
                VruTrack(
                    track_id=track_id,
                    vru_class=vru_class,
                    timestamp=_finite(row.get("timestamp"), "timestamp", line),
                    x=_finite(row.get("x"), "x", line),
                    y=_finite(row.get("y"), "y", line),
                    z=_finite(row.get("z"), "z", line, default=0.0),
                    yaw=_finite(row.get("yaw"), "yaw", line, default=0.0),
                    speed=_finite(row.get("speed"), "speed", line, default=0.0),
                    confidence=confidence,
                )
            )
    return sorted(tracks, key=lambda track: (track.timestamp, track.track_id))


def _frames(tracks: Sequence[VruTrack], tolerance: float) -> Dict[float, List[VruTrack]]:
    frames: Dict[float, List[VruTrack]] = {}
    for track in tracks:
        key = round(track.timestamp / tolerance) * tolerance if tolerance > 0.0 else track.timestamp
        frames.setdefault(round(key, 6), []).append(track)
    return frames


def _distance(a: VruTrack, b: VruTrack) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def evaluate_vru_tracks(
    ground_truth: Sequence[VruTrack],
    predicted: Sequence[VruTrack],
    max_distance: float = 1.5,
    timestamp_tolerance: float = 0.05,
) -> VruMetrics:
    """Match tracks per timestamp and return diagnostic tracking metrics.

    Matching is greedy nearest-neighbour within ``max_distance``.  MOTA here is
    a diagnostic count-based score, not the full CLEAR-MOT implementation; it is
    useful for comparing two twin runs with the same annotation convention.
    """

    if max_distance <= 0.0:
        raise ValueError("max_distance must be positive")
    gt_frames = _frames(ground_truth, timestamp_tolerance)
    pred_frames = _frames(predicted, timestamp_tolerance)
    keys = sorted(set(gt_frames) | set(pred_frames))

    matched_distances: List[float] = []
    last_distance: Dict[str, float] = {}
    last_prediction: Dict[str, str] = {}
    false_negatives = 0
    false_positives = 0
    id_switches = 0
    count_absolute_error = 0.0

    for key in keys:
        gt_frame = gt_frames.get(key, [])
        pred_frame = pred_frames.get(key, [])
        count_absolute_error += abs(len(gt_frame) - len(pred_frame))

        candidates: List[Tuple[float, int, int]] = []
        for gt_index, gt in enumerate(gt_frame):
            for pred_index, pred in enumerate(pred_frame):
                distance = _distance(gt, pred)
                if distance <= max_distance:
                    candidates.append((distance, gt_index, pred_index))
        candidates.sort(key=lambda item: item[0])

        used_gt = set()
        used_pred = set()
        for distance, gt_index, pred_index in candidates:
            if gt_index in used_gt or pred_index in used_pred:
                continue
            used_gt.add(gt_index)
            used_pred.add(pred_index)
            gt = gt_frame[gt_index]
            pred = pred_frame[pred_index]
            matched_distances.append(distance)
            last_distance[gt.track_id] = distance
            previous = last_prediction.get(gt.track_id)
            if previous is not None and previous != pred.track_id:
                id_switches += 1
            last_prediction[gt.track_id] = pred.track_id

        false_negatives += len(gt_frame) - len(used_gt)
        false_positives += len(pred_frame) - len(used_pred)

    gt_points = len(ground_truth)
    pred_points = len(predicted)
    denominator = float(gt_points) if gt_points else 1.0
    mota = 1.0 - (false_negatives + false_positives + id_switches) / denominator
    ade = sum(matched_distances) / len(matched_distances) if matched_distances else None
    fde = sum(last_distance.values()) / len(last_distance) if last_distance else None
    count_mae = count_absolute_error / len(keys) if keys else 0.0

    return VruMetrics(
        gt_points=gt_points,
        pred_points=pred_points,
        matched_points=len(matched_distances),
        false_negatives=false_negatives,
        false_positives=false_positives,
        id_switches=id_switches,
        mota=mota,
        ade_m=ade,
        fde_m=fde,
        count_mae=count_mae,
    )


def spawn_vru_tracks(
    world,
    tracks: Sequence[VruTrack],
    blueprint_filter: str = "walker.pedestrian.*",
    z_offset: float = 0.1,
):
    """Spawn one demo actor per track at its first observed pose.

    This is a deliberately small bridge for a first integration.  Production
    playback should keep one controller per actor and apply each later frame;
    the caller owns actor lifetime and must destroy the returned actors.
    CARLA is imported lazily so the metrics above remain usable without it.
    """

    import carla  # type: ignore

    blueprints = world.get_blueprint_library().filter(blueprint_filter)
    if len(blueprints) == 0:
        raise ValueError("no blueprint matches {!r}".format(blueprint_filter))

    first_track: Dict[str, VruTrack] = {}
    for track in tracks:
        first_track.setdefault(track.track_id, track)

    actors = {}
    for track_id, track in first_track.items():
        transform = carla.Transform(
            carla.Location(x=track.x, y=track.y, z=track.z + z_offset),
            carla.Rotation(yaw=track.yaw),
        )
        actor = world.try_spawn_actor(blueprints[0], transform)
        if actor is not None:
            actors[track_id] = actor
    return actors


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gt", type=Path, required=True, help="ground-truth VRU CSV")
    parser.add_argument("--pred", type=Path, required=True, help="twin-output VRU CSV")
    parser.add_argument("--max-distance", type=float, default=1.5)
    parser.add_argument("--timestamp-tolerance", type=float, default=0.05)
    parser.add_argument("--out", type=Path, default=Path("vru_metrics.json"))
    args = parser.parse_args(argv)

    metrics = evaluate_vru_tracks(
        load_vru_tracks(args.gt),
        load_vru_tracks(args.pred),
        max_distance=args.max_distance,
        timestamp_tolerance=args.timestamp_tolerance,
    )
    payload = metrics.as_dict()
    args.out.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
