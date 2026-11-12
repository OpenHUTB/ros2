"""Reaching tasks for the musculoskeletal arm: reach, shelf, draw."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

import numpy as np

from . import config as C


@dataclass
class Scene:
    name: str
    targets: List[Tuple[float, float]]
    start_q: Tuple[float, float]
    obstacles: List[Tuple[float, float, float, float]] = field(default_factory=list)
    description: str = ""
    palette: Dict[str, Tuple[int, int, int]] = field(default_factory=dict)

    # -------------------------------------------------------------- geometry
    def occupancy(self, resolution: float = C.MAP_RESOLUTION) -> np.ndarray:
        x0, x1 = C.WORK_X
        y0, y1 = C.WORK_Y
        w = int(round((x1 - x0) / resolution))
        h = int(round((y1 - y0) / resolution))
        xs = x0 + (np.arange(w) + 0.5) * resolution
        ys = y0 + (np.arange(h) + 0.5) * resolution
        gx, gy = np.meshgrid(xs, ys)
        blocked = np.zeros((h, w), dtype=bool)
        for cx, cy, hw, hh in self.obstacles:
            blocked |= (np.abs(gx - cx) <= hw) & (np.abs(gy - cy) <= hh)
        return blocked

    def coarse_map(self, resolution: float = C.PLAN_RESOLUTION) -> np.ndarray:
        return self.occupancy(resolution)

    def is_free(self, x: float, y: float, radius: float = C.EE_RADIUS) -> bool:
        for cx, cy, hw, hh in self.obstacles:
            dx = max(abs(x - cx) - hw, 0.0)
            dy = max(abs(y - cy) - hh, 0.0)
            if dx * dx + dy * dy <= radius * radius:
                return False
        return True

    def segment_hits_obstacle(self, p0: Tuple[float, float], p1: Tuple[float, float],
                              samples: int = 40) -> bool:
        for k in range(samples + 1):
            u = k / float(samples)
            x = p0[0] + (p1[0] - p0[0]) * u
            y = p0[1] + (p1[1] - p0[1]) * u
            if not self.is_free(x, y, radius=0.012):
                return True
        return False


_SCENES: Dict[str, Scene] = {}


def _mk_reach() -> Scene:
    return Scene(
        name="reach", targets=[(0.50, 0.12)], start_q=(0.20, 0.90),
        description="单目标够取：工作台右侧放置目标物，验证肌肉驱动的基础够取与保持。",
        palette={"bg": (24, 28, 34), "table": (58, 60, 66), "target": (250, 200, 90),
                 "bone": (230, 225, 215), "muscle": (200, 90, 90)},
    )


def _mk_shelf() -> Scene:
    return Scene(
        name="shelf", targets=[(0.52, 0.14)], start_q=(0.20, 0.90),
        obstacles=[(0.470, 0.340, 0.010, 0.030)],
        description="隔板绕行：目标被竖直隔板遮挡，需先抬肘越过隔板再伸向目标。",
        palette={"bg": (26, 30, 30), "table": (60, 62, 66), "target": (250, 190, 80),
                 "bone": (232, 226, 212), "muscle": (210, 100, 80)},
    )


def _mk_draw() -> Scene:
    return Scene(
        name="draw", targets=[(0.50, 0.12), (0.46, 0.33), (0.36, 0.44),
                              (0.28, 0.22), (0.46, 0.10)],
        start_q=(0.20, 0.90),
        obstacles=[(0.300, 0.500, 0.070, 0.010)],
        description="轨迹绘制：按顺序到达 5 个目标点画出闭合路径，考察连续够取与肌肉协同。",
        palette={"bg": (28, 26, 32), "table": (62, 60, 66), "target": (150, 220, 240),
                 "bone": (230, 228, 220), "muscle": (190, 120, 200)},
    )


def get_scene(name: str) -> Scene:
    if name not in _SCENES:
        builders = {"reach": _mk_reach, "shelf": _mk_shelf, "draw": _mk_draw}
        if name not in builders:
            raise KeyError(f"unknown scene {name!r}; available: {list(builders)}")
        _SCENES[name] = builders[name]()
    return _SCENES[name]


def scene_names() -> Sequence[str]:
    return tuple(C.SCENES)
