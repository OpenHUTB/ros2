"""Aerial scenes: building footprints, canyon walls and no-fly zones (NFZ).

The scene geometry is shared by three consumers:

* the MuJoCo model (collision geoms for buildings / ground),
* the occupancy map used by SLAM and the A* planner,
* the top-down renderer.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import downsample_bool


@dataclass
class Scene:
    """A square airspace with building footprints and circular no-fly zones."""

    name: str
    width: float
    height: float
    alt: float                                            # cruise altitude (m)
    start: Tuple[float, float, float, float]              # x, y, alt, yaw
    goal: Tuple[float, float, float]                      # x, y, alt
    rects: List[Tuple[float, float, float, float, float]] = field(default_factory=list)
    circles: List[Tuple[float, float, float, float]] = field(default_factory=list)
    nfz: List[Tuple[float, float, float]] = field(default_factory=list)   # cx, cy, r
    description: str = ""
    palette: Dict[str, Tuple[int, int, int]] = field(default_factory=dict)

    # ------------------------------------------------------------- occupancy
    def occupancy(self, resolution: float = C.MAP_RESOLUTION) -> np.ndarray:
        """Boolean map: True = blocked (buildings, canyon walls, outer border)."""
        w = int(round(self.width / resolution))
        h = int(round(self.height / resolution))
        grid = np.zeros((h, w), dtype=bool)
        xs = (np.arange(w) + 0.5) * resolution
        ys = (np.arange(h) + 0.5) * resolution
        gx, gy = np.meshgrid(xs, ys)
        for cx, cy, hw, hh, _height in self.rects:
            grid |= (np.abs(gx - cx) <= hw) & (np.abs(gy - cy) <= hh)
        for cx, cy, r, _height in self.circles:
            grid |= (gx - cx) ** 2 + (gy - cy) ** 2 <= r * r
        wall = int(round(0.6 / resolution))
        grid[:wall, :] = True
        grid[-wall:, :] = True
        grid[:, :wall] = True
        grid[:, -wall:] = True
        return grid

    def nfz_mask(self, resolution: float = C.MAP_RESOLUTION) -> np.ndarray:
        """Boolean mask of the no-fly zones (virtual, no collision geoms)."""
        w = int(round(self.width / resolution))
        h = int(round(self.height / resolution))
        grid = np.zeros((h, w), dtype=bool)
        xs = (np.arange(w) + 0.5) * resolution
        ys = (np.arange(h) + 0.5) * resolution
        gx, gy = np.meshgrid(xs, ys)
        for cx, cy, r in self.nfz:
            grid |= (gx - cx) ** 2 + (gy - cy) ** 2 <= r * r
        return grid

    def planning_map(self, resolution: float = C.PLAN_RESOLUTION) -> np.ndarray:
        """Blocked map for the planner: buildings + no-fly zones."""
        factor = max(1, int(round(resolution / C.MAP_RESOLUTION)))
        occ = downsample_bool(self.occupancy(C.MAP_RESOLUTION), factor)
        nfz = downsample_bool(self.nfz_mask(C.MAP_RESOLUTION), factor)
        return occ | nfz

    def coarse_map(self, resolution: float = C.PLAN_RESOLUTION) -> np.ndarray:
        return self.planning_map(resolution)

    # ---------------------------------------------------------------- NFZ math
    def nfz_clearance(self, x: float, y: float) -> float:
        """Signed distance to the closest no-fly zone boundary (negative = inside)."""
        best = float("inf")
        for cx, cy, r in self.nfz:
            d = float(np.hypot(x - cx, y - cy)) - r
            best = min(best, d)
        return best if self.nfz else float("inf")

    def inside_nfz(self, x: float, y: float) -> bool:
        return any((x - cx) ** 2 + (y - cy) ** 2 <= r * r for cx, cy, r in self.nfz)


_SCENES: Dict[str, Scene] = {}


def _city_rects() -> List[Tuple[float, float, float, float, float]]:
    """A small Manhattan grid: 4 x 4 blocks with streets between them."""
    rects = []
    block_w, street = 14.0, 10.0
    heights = [38.0, 52.0, 44.0, 60.0, 36.0, 48.0, 42.0, 56.0,
               40.0, 58.0, 46.0, 34.0, 50.0, 38.0, 62.0, 44.0]
    k = 0
    for iy in range(4):
        for ix in range(4):
            cx = 12.0 + ix * (block_w + street) + block_w / 2
            cy = 12.0 + iy * (block_w + street) + block_w / 2
            if (ix, iy) == (0, 0):
                continue  # keep the start area open
            if (ix, iy) == (3, 3):
                continue  # keep the goal pad open
            rects.append((cx, cy, block_w / 2, block_w / 2, heights[k % len(heights)]))
            k += 1
    # two low-rise warehouses on the south street
    rects.append((52.0, 8.0, 9.0, 4.0, 18.0))
    rects.append((88.0, 8.0, 7.0, 4.0, 22.0))
    return rects


def _mk_city() -> Scene:
    return Scene(
        name="city", width=130.0, height=130.0, alt=32.0,
        start=(8.0, 8.0, 32.0, 0.4), goal=(122.0, 122.0, 34.0),
        rects=_city_rects(), circles=[],
        nfz=[(65.0, 65.0, 16.0)],
        description="城市空域：4x4 街区高楼、城南低矮仓库与中心禁飞区，考察楼宇间的避障与禁飞区绕行。",
        palette={"ground": (30, 52, 42), "building": (86, 98, 104),
                 "roof": (128, 140, 146), "accent": (86, 214, 220),
                 "nfz": (206, 72, 72)},
    )


def _mk_canyon() -> Scene:
    rects = [
        (14.0, 70.0, 6.0, 70.0, 70.0),     # west wall
        (106.0, 70.0, 6.0, 70.0, 70.0),    # east wall
        (60.0, 28.0, 18.0, 5.0, 80.0),     # northern cliff
        (60.0, 112.0, 18.0, 5.0, 80.0),    # southern cliff
    ]
    circles = [
        (46.0, 58.0, 4.0, 90.0), (74.0, 82.0, 4.5, 90.0),
        (52.0, 96.0, 3.5, 90.0), (78.0, 48.0, 3.0, 90.0),
    ]
    return Scene(
        name="canyon", width=120.0, height=140.0, alt=28.0,
        start=(26.0, 70.0, 28.0, 0.0), goal=(94.0, 70.0, 30.0),
        rects=rects, circles=circles,
        nfz=[(60.0, 70.0, 12.0)],
        description="峡谷空域：两侧绝壁、南北崖壁与四座岩柱，中央禁飞区迫使无人机绕行。",
        palette={"ground": (44, 56, 40), "building": (104, 92, 74),
                 "roof": (140, 124, 100), "accent": (245, 196, 92),
                 "nfz": (214, 88, 70)},
    )


def _mk_airport() -> Scene:
    rects = [
        (30.0, 60.0, 16.0, 8.0, 26.0),     # terminal
        (30.0, 60.0, 6.0, 3.0, 44.0),      # control tower
        (120.0, 100.0, 14.0, 6.0, 30.0),   # hangar
        (120.0, 20.0, 12.0, 5.0, 24.0),
    ]
    circles = [(96.0, 62.0, 2.0, 52.0), (48.0, 96.0, 2.2, 48.0)]
    return Scene(
        name="airport", width=150.0, height=120.0, alt=34.0,
        start=(10.0, 10.0, 34.0, 0.0), goal=(140.0, 110.0, 36.0),
        rects=rects, circles=circles,
        nfz=[(55.0, 55.0, 20.0), (100.0, 38.0, 13.0)],
        description="机场空域：航站楼/塔台/机库与两块禁飞区，考察长航程下的禁飞区规避与能耗。",
        palette={"ground": (38, 50, 44), "building": (96, 104, 110),
                 "roof": (138, 146, 152), "accent": (120, 220, 200),
                 "nfz": (208, 76, 76)},
    )


def get_scene(name: str) -> Scene:
    if name not in _SCENES:
        builders = {"city": _mk_city, "canyon": _mk_canyon, "airport": _mk_airport}
        if name not in builders:
            raise KeyError(f"unknown scene {name!r}; available: {list(builders)}")
        _SCENES[name] = builders[name]()
    return _SCENES[name]


def scene_names() -> Sequence[str]:
    return tuple(C.SCENES)


# ------------------------------------------------------------------ MuJoCo XML
def mujoco_xml(scene: Scene, start=None) -> str:
    """Build the MJCF string for an aerial scene.

    Gravity is enabled (a real quadrotor), the ground plane is the world frame
    z = 0, and buildings are boxes rising from the ground.
    """
    x, y, alt, yaw = scene.start if start is None else start
    parts = [
        '<mujoco model="uav_%s">' % scene.name,
        '  <compiler angle="radian" inertiafromgeom="true"/>',
        '  <option gravity="0 0 %.2f" timestep="%.5f" integrator="implicitfast"/>'
        % (-C.GRAVITY, C.PHYS_DT),
        '  <default><geom friction="0.8 0.1 0.1" density="1000"/></default>',
        '  <worldbody>',
        '    <geom name="ground" type="plane" pos="0 0 0" size="%f %f 0.1" rgba="0.18 0.30 0.24 1" group="1"/>'
        % (scene.width, scene.height),
    ]
    # buildings (boxes from z=0 upwards)
    for i, (cx, cy, hw, hh, hgt) in enumerate(scene.rects):
        parts.append(
            '    <geom name="bld_%d" type="box" pos="%f %f %f" size="%f %f %f" rgba="0.35 0.38 0.40 1" group="1"/>'
            % (i, cx, cy, hgt / 2.0, hw, hh, hgt / 2.0))
    for i, (cx, cy, r, hgt) in enumerate(scene.circles):
        parts.append(
            '    <geom name="rock_%d" type="cylinder" pos="%f %f %f" size="%f %f" rgba="0.40 0.38 0.32 1" group="1"/>'
            % (i, cx, cy, hgt / 2.0, r, hgt / 2.0))
    # outer walls (so the airspace is bounded)
    H = 90.0
    W, L = scene.width, scene.height
    parts += [
        '    <geom name="wall_x0" type="box" pos="0 0 %f" size="0.6 %f %f" group="1"/>' % (H / 2, L, H / 2),
        '    <geom name="wall_x1" type="box" pos="%f 0 %f" size="0.6 %f %f" group="1"/>' % (W, H / 2, L, H / 2),
        '    <geom name="wall_y0" type="box" pos="%f 0 %f" size="%f 0.6 %f" group="1"/>' % (W / 2, H / 2, W / 2, H / 2),
        '    <geom name="wall_y1" type="box" pos="%f %f %f" size="%f 0.6 %f" group="1"/>' % (W / 2, L, H / 2, W / 2, H / 2),
        # UAV body
        '    <body name="uav" pos="%f %f %f">' % (x, y, alt),
        '      <freejoint name="uav_free"/>',
        '      <geom name="hull" type="box" size="%f %f %f" mass="%f" rgba="0.95 0.62 0.18 1" group="2"/>'
        % (C.BODY_HALF[0], C.BODY_HALF[1], C.BODY_HALF[2], C.MASS),
        '    </body>',
        '  </worldbody>',
        '</mujoco>',
    ]
    return "\n".join(parts)
