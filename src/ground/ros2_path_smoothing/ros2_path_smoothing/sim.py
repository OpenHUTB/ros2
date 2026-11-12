"""Grid maps and metrics for the path-smoothing module."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

try:
    from .planner import (Grid, astar, catmull_rom, clearance, curvature,
                          path_length, smooth_path)
except ImportError:
    from planner import (Grid, astar, catmull_rom, clearance, curvature,
                         path_length, smooth_path)


@dataclass
class MapSpec:
    name: str
    start: Tuple[int, int]
    goal: Tuple[int, int]
    grid: Grid


def blank(width: int = 200, height: int = 100) -> np.ndarray:
    return np.zeros((height, width), dtype=bool)


def rect(grid: np.ndarray, x0: int, y0: int, x1: int, y1: int) -> None:
    grid[max(0, y0):min(grid.shape[0], y1), max(0, x0):min(grid.shape[1], x1)] = True


def circle(grid: np.ndarray, cx: int, cy: int, r: int) -> None:
    yy, xx = np.ogrid[:grid.shape[0], :grid.shape[1]]
    grid[(xx - cx) ** 2 + (yy - cy) ** 2 <= r * r] = True


def corridor() -> MapSpec:
    grid = blank()
    rect(grid, 0, 0, 200, 2)
    rect(grid, 0, 98, 200, 100)
    rect(grid, 0, 0, 2, 100)
    rect(grid, 198, 0, 200, 100)
    for x, y, r in ((60, 50, 6), (110, 35, 6), (150, 60, 6)):
        circle(grid, x, y, r)
    return MapSpec("corridor", (6, 50), (193, 50), Grid(grid))


def doorway() -> MapSpec:
    grid = blank()
    rect(grid, 0, 0, 200, 2)
    rect(grid, 0, 98, 200, 100)
    rect(grid, 0, 0, 2, 100)
    rect(grid, 198, 0, 200, 100)
    rect(grid, 100, 2, 104, 42)
    rect(grid, 100, 58, 104, 98)
    return MapSpec("doorway", (6, 20), (193, 80), Grid(grid))


def clutter(seed: int = 7) -> MapSpec:
    grid = blank()
    rect(grid, 0, 0, 200, 2)
    rect(grid, 0, 98, 200, 100)
    rect(grid, 0, 0, 2, 100)
    rect(grid, 198, 0, 200, 100)
    rng = random.Random(seed)
    for _ in range(14):
        circle(grid, rng.randint(15, 185), rng.randint(10, 90),
               rng.randint(3, 7))
    return MapSpec("clutter", (6, 10), (193, 90), Grid(grid))


SCENARIOS = {"corridor": corridor, "doorway": doorway, "clutter": clutter}


def evaluate(spec: MapSpec) -> Dict[str, object]:
    raw_cells = astar(spec.grid, spec.start, spec.goal)
    if not raw_cells:
        return {"map": spec.name, "ok": False}
    raw = np.asarray([spec.grid.to_world(c) for c in raw_cells])
    smooth, safe = smooth_path(spec.grid, raw_cells)
    k_raw, k_smooth = curvature(raw), curvature(smooth)
    return {
        "map": spec.name,
        "ok": True,
        "raw_points": len(raw),
        "smooth_points": len(smooth),
        "raw_length_m": round(path_length(raw), 3),
        "smooth_length_m": round(path_length(smooth), 3),
        "raw_max_curvature": round(float(k_raw.max()) if k_raw.size else 0.0, 3),
        "smooth_max_curvature": round(float(k_smooth.max()) if k_smooth.size else 0.0, 3),
        "raw_min_clearance_m": round(clearance(spec.grid, raw), 3),
        "smooth_min_clearance_m": round(clearance(spec.grid, smooth), 3),
        "smooth_safe": bool(safe),
    }


def main() -> None:
    print(f"{'map':10s} {'ok':>3} {'len_raw':>8} {'len_sm':>8} "
          f"{'k_raw':>7} {'k_sm':>7} {'clear':>7} {'safe':>5}")
    for factory in SCENARIOS.values():
        row = evaluate(factory())
        print(f"{row['map']:10s} {str(row.get('ok')):>3} "
              f"{row.get('raw_length_m', 0):>8} {row.get('smooth_length_m', 0):>8} "
              f"{row.get('raw_max_curvature', 0):>7} "
              f"{row.get('smooth_max_curvature', 0):>7} "
              f"{row.get('smooth_min_clearance_m', 0):>7} "
              f"{str(row.get('smooth_safe')):>5}")


if __name__ == "__main__":
    main()

