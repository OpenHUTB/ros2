"""Indoor scenes for the humanoid walking robot: hall, warehouse, obstacle course."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import config as C


@dataclass
class Scene:
    """A rectangular indoor floor plan with walls, pillars, crates and carts."""

    name: str
    width: float
    height: float
    start: Tuple[float, float, float, float]
    goal: Tuple[float, float, float]
    walls: List[Tuple[float, float, float, float]] = field(default_factory=list)
    obstacles: List[Tuple[float, float, float, float]] = field(default_factory=list)
    pillars: List[Tuple[float, float, float]] = field(default_factory=list)
    carts: List[Dict[str, float]] = field(default_factory=list)
    description: str = ""
    palette: Dict[str, Tuple[int, int, int]] = field(default_factory=dict)

    # ------------------------------------------------------------- floor plan
    def _grid(self, resolution: float):
        w = int(round(self.width / resolution))
        h = int(round(self.height / resolution))
        xs = (np.arange(w) + 0.5) * resolution
        ys = (np.arange(h) + 0.5) * resolution
        gx, gy = np.meshgrid(xs, ys)
        return gx, gy, w, h

    def drivable_mask(self, resolution: float = C.MAP_RESOLUTION) -> np.ndarray:
        gx, gy, w, h = self._grid(resolution)
        return np.ones((h, w), dtype=bool)

    def occupancy(self, resolution: float = C.MAP_RESOLUTION) -> np.ndarray:
        gx, gy, w, h = self._grid(resolution)
        blocked = np.zeros((h, w), dtype=bool)
        for cx, cy, hw, hh in self.walls:
            blocked |= (np.abs(gx - cx) <= hw) & (np.abs(gy - cy) <= hh)
        for cx, cy, hw, hh in self.obstacles:
            blocked |= (np.abs(gx - cx) <= hw) & (np.abs(gy - cy) <= hh)
        for cx, cy, r in self.pillars:
            blocked |= (gx - cx) ** 2 + (gy - cy) ** 2 <= r * r
        return blocked

    def coarse_map(self, resolution: float = C.PLAN_RESOLUTION) -> np.ndarray:
        return self.occupancy(resolution)

    # ---------------------------------------------------------------- geometry
    def is_free(self, x: float, y: float, radius: float = C.ROBOT_RADIUS) -> bool:
        if x < radius or y < radius or x > self.width - radius or y > self.height - radius:
            return False
        for cx, cy, hw, hh in self.walls:
            dx = max(abs(x - cx) - hw, 0.0)
            dy = max(abs(y - cy) - hh, 0.0)
            if dx * dx + dy * dy <= radius * radius:
                return False
        for cx, cy, hw, hh in self.obstacles:
            dx = max(abs(x - cx) - hw, 0.0)
            dy = max(abs(y - cy) - hh, 0.0)
            if dx * dx + dy * dy <= radius * radius:
                return False
        for cx, cy, r in self.pillars:
            if (x - cx) ** 2 + (y - cy) ** 2 <= (radius + r) ** 2:
                return False
        return True

    def cart_positions(self, t: float) -> List[Tuple[float, float, float, float]]:
        """Moving carts: linear back-and-forth between two points."""
        out = []
        for cart in self.carts:
            x0, y0 = float(cart["x0"]), float(cart["y0"])
            x1, y1 = float(cart["x1"]), float(cart["y1"])
            speed = float(cart.get("speed", 0.30))
            phase = float(cart.get("phase", 0.0))
            length = max(1e-3, float(np.hypot(x1 - x0, y1 - y0)))
            period = 2.0 * length / speed
            u = ((t / period) + phase) % 1.0
            s = 2.0 * u if u < 0.5 else 2.0 * (1.0 - u)
            out.append((x0 + (x1 - x0) * s, y0 + (y1 - y0) * s,
                        float(cart.get("hw", 0.45)), float(cart.get("hh", 0.30))))
        return out


_SCENES: Dict[str, Scene] = {}


def _boundary(scene_w: float, scene_h: float, t: float = 0.4):
    return [
        (scene_w / 2.0, -t / 2.0, scene_w / 2.0, t / 2.0),
        (scene_w / 2.0, scene_h + t / 2.0, scene_w / 2.0, t / 2.0),
        (-t / 2.0, scene_h / 2.0, t / 2.0, scene_h / 2.0),
        (scene_w + t / 2.0, scene_h / 2.0, t / 2.0, scene_h / 2.0),
    ]


def _mk_hall() -> Scene:
    walls = _boundary(30.0, 12.0)
    walls += [(10.0, 3.2, 0.35, 3.2), (20.0, 8.8, 0.35, 3.2)]
    pillars = [(7.0, 6.0, 0.55), (15.0, 6.0, 0.55), (23.0, 6.0, 0.55)]
    return Scene(
        name="hall", width=30.0, height=12.0,
        start=(2.0, 6.0, 0.0, 0.0), goal=(28.0, 6.0, 0.0),
        walls=walls, pillars=pillars,
        description="办公楼走廊：两段隔墙、三根立柱，考察窄通道步行与绕柱导航。",
        palette={"floor": (42, 46, 52), "wall": (120, 126, 136),
                 "accent": (90, 210, 200), "robot": (235, 190, 90)},
    )


def _mk_warehouse() -> Scene:
    walls = _boundary(24.0, 20.0)
    obstacles = [
        (8.0, 6.0, 3.2, 0.7), (16.0, 6.0, 3.2, 0.7),
        (8.0, 14.0, 3.2, 0.7), (16.0, 14.0, 3.2, 0.7),
    ]
    carts = [
        {"x0": 3.0, "y0": 10.0, "x1": 21.0, "y1": 10.0, "speed": 0.35, "phase": 0.0},
    ]
    return Scene(
        name="warehouse", width=24.0, height=20.0,
        start=(2.0, 2.0, 0.0, 0.0), goal=(22.0, 18.0, 0.0),
        walls=walls, obstacles=obstacles, carts=carts,
        description="仓储货架区：四组货架、横向移动的搬运车，考察货架间通道与动态避障。",
        palette={"floor": (50, 48, 44), "wall": (132, 124, 108),
                 "accent": (240, 170, 80), "robot": (120, 210, 240)},
    )


def _mk_obstacle() -> Scene:
    walls = _boundary(20.0, 20.0)
    crates = [
        (6.0, 5.0, 1.0, 0.8), (11.0, 4.0, 0.8, 1.2), (15.0, 7.0, 1.0, 0.9),
        (5.0, 11.0, 1.2, 0.8), (9.0, 13.0, 0.9, 0.9), (14.0, 12.0, 1.1, 0.8),
        (7.0, 17.0, 1.0, 0.8), (16.0, 17.0, 0.9, 1.0),
    ]
    pillars = [(12.0, 9.0, 0.5)]
    return Scene(
        name="obstacle", width=20.0, height=20.0,
        start=(2.0, 2.0, 0.0, 0.0), goal=(18.0, 18.0, 0.0),
        walls=walls, obstacles=crates, pillars=pillars,
        description="障碍训练场：散布木箱与立柱，考察密集障碍下的规划与步行稳定性。",
        palette={"floor": (46, 52, 46), "wall": (118, 132, 118),
                 "accent": (150, 230, 130), "robot": (250, 160, 180)},
    )


def get_scene(name: str) -> Scene:
    if name not in _SCENES:
        builders = {"hall": _mk_hall, "warehouse": _mk_warehouse,
                    "obstacle": _mk_obstacle}
        if name not in builders:
            raise KeyError(f"unknown scene {name!r}; available: {list(builders)}")
        _SCENES[name] = builders[name]()
    return _SCENES[name]


def scene_names() -> Sequence[str]:
    return tuple(C.SCENES)
