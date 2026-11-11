"""Paths and metrics for the velocity-profiling module."""

from __future__ import annotations

import math
from typing import Dict, List, Sequence, Tuple

import numpy as np

try:
    from .profile import constant_profile, speed_profile
except ImportError:
    from profile import constant_profile, speed_profile


def circle_points(radius: float = 1.2, count: int = 300) -> np.ndarray:
    t = np.linspace(0.0, 2.0 * math.pi, count)
    return np.column_stack([radius * np.cos(t), radius * np.sin(t)])


def slalom_points(count: int = 300) -> np.ndarray:
    x = np.linspace(0.0, 10.0, count)
    y = 1.2 * np.sin(2.0 * math.pi * x / 5.0)
    return np.column_stack([x, y])


def square_points(side: float = 5.0, spacing: float = 0.05) -> np.ndarray:
    corners = [(0, 0), (side, 0), (side, side), (0, side), (0, 0)]
    pts = []
    for (x0, y0), (x1, y1) in zip(corners[:-1], corners[1:]):
        n = int(math.hypot(x1 - x0, y1 - y0) / spacing)
        for i in range(n):
            pts.append((x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n))
    pts.append(corners[-1])
    return np.asarray(pts, dtype=float)


PATHS = {"circle": circle_points, "slalom": slalom_points, "square": square_points}


def evaluate(name: str, points: Sequence[Sequence[float]]) -> Dict[str, object]:
    base = constant_profile(points, v=1.0)
    prof = speed_profile(points, v_max=1.0, a_max=0.8, a_lat_max=0.5)
    return {
        "path": name,
        "points": len(points),
        "length_m": round(float(np.sum(np.linalg.norm(
            np.diff(np.asarray(points), axis=0), axis=1))), 3),
        "constant_time_s": round(base.duration, 3),
        "profile_time_s": round(prof.duration, 3),
        "constant_max_lat": round(base.max_lateral_accel, 3),
        "profile_max_lat": round(prof.max_lateral_accel, 3),
        "profile_max_accel": round(prof.max_accel, 3),
        "min_speed": round(float(prof.speed.min()), 3),
        "max_speed": round(float(prof.speed.max()), 3),
    }


def run_benchmark() -> List[Dict[str, object]]:
    return [evaluate(name, factory()) for name, factory in PATHS.items()]


def main() -> None:
    print(f"{'path':8s} {'len':>7} {'t_const':>8} {'t_prof':>7} "
          f"{'alat_c':>7} {'alat_p':>7} {'amax_p':>7}")
    for row in run_benchmark():
        print(f"{row['path']:8s} {row['length_m']:>7} {row['constant_time_s']:>8} "
              f"{row['profile_time_s']:>7} {row['constant_max_lat']:>7} "
              f"{row['profile_max_lat']:>7} {row['profile_max_accel']:>7}")


if __name__ == "__main__":
    main()
