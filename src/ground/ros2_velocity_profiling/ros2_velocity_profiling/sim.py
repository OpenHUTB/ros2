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


def square_points(side: float = 5.0, radius: float = 0.8,
                  spacing: float = 0.05) -> np.ndarray:
    """Rounded square: straight edges + quarter-circle corners (radius 0.8 m)."""
    pts = []

    def line(a, b):
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / spacing))
        for i in range(n):
            pts.append((a[0] + (b[0] - a[0]) * i / n,
                        a[1] + (b[1] - a[1]) * i / n))

    def arc(cx, cy, a0, a1):
        n = max(2, int(abs(a1 - a0) * radius / spacing))
        for i in range(n):
            a = a0 + (a1 - a0) * i / n
            pts.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))

    r = radius
    line((r, 0.0), (side - r, 0.0))
    arc(side - r, r, -math.pi / 2, 0.0)
    line((side, r), (side, side - r))
    arc(side - r, side - r, 0.0, math.pi / 2)
    line((side - r, side), (r, side))
    arc(r, side - r, math.pi / 2, math.pi)
    line((0.0, side - r), (0.0, r))
    arc(r, r, math.pi, 1.5 * math.pi)
    pts.append((r, 0.0))
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

