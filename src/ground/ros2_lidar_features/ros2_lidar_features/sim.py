"""Synthetic corridor scans and metrics for line/door extraction."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np

try:
    from .features import detect_doors, extract_lines, points_from_scan
except ImportError:
    from features import detect_doors, extract_lines, points_from_scan


@dataclass
class Segment:
    x1: float
    y1: float
    x2: float
    y2: float


def raycast(segments: Sequence[Segment], n_beams: int = 360,
            max_range: float = 8.0) -> np.ndarray:
    angles = np.linspace(-math.pi, math.pi, n_beams, endpoint=False)
    dirs = np.column_stack([np.cos(angles), np.sin(angles)])
    ranges = np.full(n_beams, np.inf)
    for s in segments:
        sx, sy = s.x2 - s.x1, s.y2 - s.y1
        qx, qy = s.x1, s.y1
        denom = dirs[:, 0] * sy - dirs[:, 1] * sx
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (qx * sy - qy * sx) / denom
            u = (qx * dirs[:, 1] - qy * dirs[:, 0]) / denom
        valid = (np.abs(denom) > 1e-12) & (t > 0.0) & (u >= 0.0) & (u <= 1.0)
        ranges = np.minimum(ranges, np.where(valid, t, np.inf))
    return np.where(np.isfinite(ranges), ranges, np.inf)


def corridor() -> Dict[str, object]:
    segments = [
        Segment(-3.0, 1.2, 3.0, 1.2),    # left wall before door
        Segment(4.0, 1.2, 10.0, 1.2),    # left wall after door
        Segment(-3.0, -1.2, 10.0, -1.2),  # right wall
    ]
    return {"name": "door", "segments": segments, "door_width": 1.0}


def closed_corridor() -> Dict[str, object]:
    segments = [
        Segment(-3.0, 1.2, 10.0, 1.2),
        Segment(-3.0, -1.2, 10.0, -1.2),
    ]
    return {"name": "closed", "segments": segments, "door_width": 0.0}


def noisy_corridor() -> Dict[str, object]:
    spec = corridor()
    spec["name"] = "noisy"
    spec["noise"] = 0.01
    return spec


def evaluate(spec: Dict[str, object]) -> Dict[str, object]:
    ranges = raycast(spec["segments"], n_beams=720)  # type: ignore[arg-type]
    if spec.get("noise"):
        rng = np.random.default_rng(7)
        noise = rng.normal(0.0, float(spec["noise"]), size=ranges.size)
        ranges = np.where(np.isfinite(ranges), ranges + noise, ranges)
    points = points_from_scan(ranges, -math.pi, 2.0 * math.pi / len(ranges),
                              max_range=8.0)
    lines = extract_lines(points, angle_increment=2.0 * math.pi / len(ranges))
    doors = detect_doors(lines)
    widths = [math.hypot(d[2] - d[0], d[3] - d[1]) for d in doors]
    return {
        "scenario": spec["name"],
        "lines": len(lines),
        "mean_residual_m": round(float(np.mean([l.residual for l in lines]))
                                 if lines else 0.0, 4),
        "doors": len(doors),
        "door_widths_m": [round(w, 3) for w in widths],
        "expected_width_m": spec["door_width"],
        "width_error_m": (round(abs(widths[0] - float(spec["door_width"])), 3)
                          if doors else None),
    }


def run_benchmark() -> List[Dict[str, object]]:
    return [evaluate(corridor()), evaluate(noisy_corridor()), evaluate(closed_corridor())]


def main() -> None:
    for row in run_benchmark():
        print(row)


if __name__ == "__main__":
    main()




