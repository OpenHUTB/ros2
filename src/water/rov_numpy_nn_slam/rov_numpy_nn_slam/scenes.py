"""Underwater scenes: geometry shared by the MuJoCo model, the map and rendering."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import downsample_bool


@dataclass
class Scene:
    """A rectangular water volume with box / cylinder obstacles."""

    name: str
    width: float
    height: float
    depth: float                 # water depth (m)
    start: Tuple[float, float, float, float]   # x, y, depth, yaw
    goal: Tuple[float, float, float]           # x, y, depth
    rects: List[Tuple[float, float, float, float]] = field(default_factory=list)
    circles: List[Tuple[float, float, float]] = field(default_factory=list)
    description: str = ""
    palette: Dict[str, Tuple[int, int, int]] = field(default_factory=dict)

    # ------------------------------------------------------------- occupancy
    def occupancy(self, resolution: float = C.MAP_RESOLUTION) -> np.ndarray:
        """Boolean map: True = occupied (or outside the walls)."""
        w = int(round(self.width / resolution))
        h = int(round(self.height / resolution))
        grid = np.zeros((h, w), dtype=bool)
        xs = (np.arange(w) + 0.5) * resolution
        ys = (np.arange(h) + 0.5) * resolution
        gx, gy = np.meshgrid(xs, ys)
        for cx, cy, hw, hh in self.rects:
            grid |= (np.abs(gx - cx) <= hw) & (np.abs(gy - cy) <= hh)
        for cx, cy, r in self.circles:
            grid |= (gx - cx) ** 2 + (gy - cy) ** 2 <= r * r
        # outer walls, two cells thick
        wall = int(round(0.12 / resolution))
        grid[:wall, :] = True
        grid[-wall:, :] = True
        grid[:, :wall] = True
        grid[:, -wall:] = True
        return grid

    def coarse_map(self, resolution: float = C.PLAN_RESOLUTION) -> np.ndarray:
        factor = max(1, int(round(resolution / C.MAP_RESOLUTION)))
        return downsample_bool(self.occupancy(C.MAP_RESOLUTION), factor)


_SCENES: Dict[str, Scene] = {}


def _mk_pool() -> Scene:
    rects = [
        (7.0, 5.5, 0.18, 0.18),          # lane rope anchor blocks
        (10.6, 7.4, 1.30, 0.16),         # diving platform leg
    ]
    circles = [
        (3.4, 7.6, 0.45), (5.6, 2.8, 0.45), (7.9, 8.2, 0.55),
        (9.4, 3.6, 0.40), (11.4, 5.2, 0.50), (12.4, 8.6, 0.35),
    ]
    return Scene(
        name="pool", width=14.0, height=11.0, depth=4.2,
        start=(1.6, 1.6, 1.6, 0.0), goal=(12.4, 9.4, 2.2),
        rects=rects, circles=circles,
        description="训练泳池：规则池壁、泳道浮标锚块与六根立柱，适合基础遥控与建图。",
        palette={"water": (16, 46, 68), "floor": (26, 66, 92),
                 "obstacle": (78, 116, 138), "accent": (86, 214, 220)},
    )


def _mk_harbor() -> Scene:
    rects = [
        (6.5, 14.0, 4.2, 0.55),          # quay wall
        (13.6, 14.0, 2.6, 0.55),
        (17.2, 2.2, 2.2, 0.5),           # floating dock
        (3.0, 8.4, 0.9, 0.35),           # pontoon
    ]
    circles = [
        (2.6, 5.0, 0.30), (5.2, 11.4, 0.32), (8.8, 9.6, 0.28),
        (10.2, 6.2, 0.35), (12.8, 9.0, 0.30), (15.4, 7.2, 0.28),
        (16.6, 11.6, 0.32), (7.4, 4.2, 0.26), (4.2, 2.4, 0.28),
    ]
    return Scene(
        name="harbor", width=20.0, height=15.0, depth=6.0,
        start=(2.4, 2.4, 2.6, 0.6), goal=(17.6, 12.6, 3.4),
        rects=rects, circles=circles,
        description="港池：码头岸壁、浮桥与密集桩柱，用于多目标长航程导航测试。",
        palette={"water": (12, 38, 58), "floor": (24, 56, 78),
                 "obstacle": (92, 104, 112), "accent": (245, 196, 92)},
    )


def _mk_cave() -> Scene:
    circles = [
        (4.2, 7.0, 1.10), (4.2, 3.6, 0.95), (4.2, 10.4, 0.90),
        (7.0, 5.0, 1.35), (7.0, 9.2, 1.20), (7.0, 12.8, 0.85),
        (9.8, 7.2, 1.05), (9.8, 2.2, 1.15), (9.8, 11.4, 0.95),
        (12.6, 4.6, 1.25), (12.6, 9.0, 1.10), (12.6, 13.2, 0.80),
        (15.2, 7.0, 1.00), (15.2, 11.6, 0.90),
    ]
    rects = [(17.0, 2.2, 1.2, 0.6)]
    return Scene(
        name="cave", width=18.0, height=14.0, depth=5.0,
        start=(1.6, 7.0, 2.0, 0.0), goal=(16.4, 7.0, 2.6),
        rects=rects, circles=circles,
        description="水下洞穴：不规则岩体形成蜿蜒通道，用于窄通道避障与鲁棒建图。",
        palette={"water": (10, 30, 46), "floor": (20, 44, 60),
                 "obstacle": (104, 92, 78), "accent": (250, 148, 96)},
    )


def get_scene(name: str) -> Scene:
    if name not in _SCENES:
        builders = {"pool": _mk_pool, "harbor": _mk_harbor, "cave": _mk_cave}
        if name not in builders:
            raise KeyError(f"unknown scene {name!r}; available: {list(builders)}")
        _SCENES[name] = builders[name]()
    return _SCENES[name]


def scene_names() -> Sequence[str]:
    return tuple(C.SCENES)


# ------------------------------------------------------------------ MuJoCo XML
def mujoco_xml(scene: Scene, start=None) -> str:
    """Build the MJCF string for a scene.

    Gravity is disabled: the vehicle model applies weight, buoyancy and fluid
    drag explicitly (see ``sim.py``), which keeps the underwater dynamics
    equations visible and documentable.
    """
    x, y, depth, yaw = scene.start if start is None else start
    H = scene.depth
    W = scene.width
    L = scene.height
    parts = [
        '<mujoco model="rov_%s">' % scene.name,
        '  <compiler angle="radian" inertiafromgeom="true"/>',
        '  <option gravity="0 0 0" timestep="%.5f" integrator="implicitfast"/>' % C.PHYS_DT,
        '  <default><geom friction="0.8 0.1 0.1" density="1000"/></default>',
        '  <worldbody>',
        # water surface at z = 0, seabed at z = -H, four vertical walls
        '    <geom name="seabed" type="plane" pos="0 0 %f" size="%f %f 0.1" rgba="0.12 0.25 0.32 1" group="1"/>' % (-H, W, L),
        '    <geom name="surface" type="box" pos="0 0 0.2" size="%f %f 0.2" rgba="0.2 0.5 0.7 0.2" group="1"/>' % (W, L),
        '    <geom name="wall_x0" type="box" pos="0 0 %f" size="0.3 %f %f" group="1"/>' % (-H / 2, L, H / 2),
        '    <geom name="wall_x1" type="box" pos="%f 0 %f" size="0.3 %f %f" group="1"/>' % (W, -H / 2, L, H / 2),
        '    <geom name="wall_y0" type="box" pos="%f 0 %f" size="%f 0.3 %f" group="1"/>' % (W / 2, -H / 2, W / 2, H / 2),
        '    <geom name="wall_y1" type="box" pos="%f %f %f" size="%f 0.3 %f" group="1"/>' % (W / 2, L, -H / 2, W / 2, H / 2),
    ]
    # obstacles rise from the seabed past the water surface
    for i, (cx, cy, hw, hh) in enumerate(scene.rects):
        parts.append('    <geom name="rect_%d" type="box" pos="%f %f %f" size="%f %f %f" group="1"/>'
                     % (i, cx, cy, -H / 2, hw, hh, H / 2))
    for i, (cx, cy, r) in enumerate(scene.circles):
        parts.append('    <geom name="cyl_%d" type="cylinder" pos="%f %f %f" size="%f %f" group="1"/>'
                     % (i, cx, cy, -H / 2, r, H / 2))
    # ROV hull (group 2 so sonar rays only see group 1)
    parts += [
        '    <body name="rov" pos="%f %f %f">' % (x, y, -depth),
        '      <freejoint name="rov_free"/>',
        '      <geom name="hull" type="box" size="%f %f %f" mass="%f" rgba="0.85 0.55 0.15 1" group="2"/>'
        % (C.HULL_HALF[0], C.HULL_HALF[1], C.HULL_HALF[2], C.MASS),
        '    </body>',
        '  </worldbody>',
        '</mujoco>',
    ]
    return "\n".join(parts)
