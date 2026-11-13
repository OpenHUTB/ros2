"""Synthetic maps and metrics for the costmap-inflation module."""

from __future__ import annotations

import math
from typing import Dict

import numpy as np

try:
    from .inflation import inflate, summary
except ImportError:
    from inflation import inflate, summary


def demo_occupancy(width: int = 200, height: int = 100) -> np.ndarray:
    occ = np.zeros((height, width), dtype=np.int8)
    occ[0:2, :] = 100
    occ[-2:, :] = 100
    occ[:, 0:2] = 100
    occ[:, -2:] = 100
    yy, xx = np.ogrid[:height, :width]
    for cx, cy, r in ((50, 50, 8), (110, 35, 7), (150, 65, 6)):
        occ[(xx - cx) ** 2 + (yy - cy) ** 2 <= r * r] = 100
    return occ


def run_demo() -> Dict[str, float]:
    occ = demo_occupancy()
    result = inflate(occ, resolution=0.1, inflation_radius=0.6,
                     inscribed_radius=0.25, cost_scaling=3.0)
    return summary(result, resolution=0.1)


def main() -> None:
    for key, value in run_demo().items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
