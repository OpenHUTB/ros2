"""Road-network scenes: lanes, intersections, traffic lights and pedestrians.

The scene owns the drivable geometry.  The 2-D bicycle simulator uses it for
the lidar (curb / obstacle raymarching), collision checks, traffic-light state
and the top-down renderer; the planner uses its occupancy grid.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import config as C
from .geometry import downsample_bool


@dataclass
class Scene:
    """A rectilinear road network with lights, parked cars and pedestrians."""

    name: str
    width: float
    height: float
    start: Tuple[float, float, float, float]        # x, y, z(0), yaw
    goal: Tuple[float, float, float]                # x, y, z(0)
    roads: List[Tuple[float, float, float, float]] = field(default_factory=list)
    parked: List[Tuple[float, float, float, float]] = field(default_factory=list)
    cones: List[Tuple[float, float, float]] = field(default_factory=list)
    barriers: List[Tuple[float, float, float, float]] = field(default_factory=list)
    lights: List[Dict[str, float]] = field(default_factory=list)
    pedestrians: List[Dict[str, float]] = field(default_factory=list)
    description: str = ""
    palette: Dict[str, Tuple[int, int, int]] = field(default_factory=dict)

    # ------------------------------------------------------------- drivability
    def drivable_mask(self, resolution: float = C.MAP_RESOLUTION) -> np.ndarray:
        w = int(round(self.width / resolution))
        h = int(round(self.height / resolution))
        xs = (np.arange(w) + 0.5) * resolution
        ys = (np.arange(h) + 0.5) * resolution
        gx, gy = np.meshgrid(xs, ys)
        mask = np.zeros((h, w), dtype=bool)
        for cx, cy, hw, hh in self.roads:
            mask |= (np.abs(gx - cx) <= hw) & (np.abs(gy - cy) <= hh)
        return mask

    def occupancy(self, resolution: float = C.MAP_RESOLUTION) -> np.ndarray:
        """Boolean blocked map: off-road, parked cars, cones and barriers."""
        mask = self.drivable_mask(resolution)
        w = int(round(self.width / resolution))
        h = int(round(self.height / resolution))
        xs = (np.arange(w) + 0.5) * resolution
        ys = (np.arange(h) + 0.5) * resolution
        gx, gy = np.meshgrid(xs, ys)
        blocked = ~mask
        for cx, cy, hw, hh in self.parked:
            blocked |= (np.abs(gx - cx) <= hw) & (np.abs(gy - cy) <= hh)
        for cx, cy, hw, hh in self.barriers:
            blocked |= (np.abs(gx - cx) <= hw) & (np.abs(gy - cy) <= hh)
        for cx, cy, r in self.cones:
            blocked |= (gx - cx) ** 2 + (gy - cy) ** 2 <= r * r
        return blocked

    def coarse_map(self, resolution: float = C.PLAN_RESOLUTION) -> np.ndarray:
        """Blocked map at exactly ``resolution`` (off-road + obstacles)."""
        return self.occupancy(resolution)

    # ---------------------------------------------------------------- geometry
    def is_drivable(self, x: float, y: float) -> bool:
        if x < 0.0 or y < 0.0 or x > self.width or y > self.height:
            return False
        for cx, cy, hw, hh in self.roads:
            if abs(x - cx) <= hw and abs(y - cy) <= hh:
                return True
        return False

    def obstacle_rects(self) -> List[Tuple[float, float, float, float]]:
        return list(self.parked) + list(self.barriers)

    # ------------------------------------------------------------- traffic light
    def light_state(self, light: Dict[str, float], t: float) -> str:
        phase = (t + float(light.get("offset", 0.0))) % C.LIGHT_CYCLE
        if phase < C.LIGHT_GREEN:
            return "green"
        if phase < C.LIGHT_GREEN + C.LIGHT_YELLOW:
            return "yellow"
        return "red"

    def lights_for_pose(self, x: float, y: float, yaw: float,
                        t: float) -> List[Dict[str, float]]:
        """Traffic lights on the current approach within 40 m, nearest first."""
        out: List[Dict[str, float]] = []
        for light in self.lights:
            lx, ly = float(light["x"]), float(light["y"])
            axis = str(light.get("axis", "x"))
            direction = float(light.get("dir", 1.0))
            if axis == "x":
                lateral = abs(y - ly)
                ahead = (lx - x) * direction
            else:
                lateral = abs(x - lx)
                ahead = (ly - y) * direction
            if lateral > 3.5 or ahead < -1.0 or ahead > 40.0:
                continue
            yaw_ok = (abs(np.sin(yaw)) < 0.5) if axis == "x" else (abs(np.cos(yaw)) < 0.5)
            if not yaw_ok:
                continue
            out.append({
                "x": lx, "y": ly, "axis": axis, "dir": direction,
                "distance": float(ahead), "state": self.light_state(light, t),
                "stop": float(C.LIGHT_CYCLE), "offset": float(light.get("offset", 0.0)),
            })
        out.sort(key=lambda d: d["distance"])
        return out

    def nearest_light(self, x: float, y: float, yaw: float, t: float
                      ) -> Optional[Dict[str, float]]:
        lights = self.lights_for_pose(x, y, yaw, t)
        return lights[0] if lights else None

    # --------------------------------------------------------------- pedestrians
    def pedestrian_positions(self, t: float) -> List[Tuple[float, float]]:
        out = []
        for ped in self.pedestrians:
            x0, y0 = float(ped["x0"]), float(ped["y0"])
            x1, y1 = float(ped["x1"]), float(ped["y1"])
            length = max(1e-3, float(np.hypot(x1 - x0, y1 - y0)))
            speed = float(ped.get("speed", 1.2))
            phase = float(ped.get("phase", 0.0))
            period = 2.0 * length / speed
            u = ((t / period) + phase) % 1.0
            s = 2.0 * u if u < 0.5 else 2.0 * (1.0 - u)
            out.append((x0 + (x1 - x0) * s, y0 + (y1 - y0) * s))
        return out


_SCENES: Dict[str, Scene] = {}


def _mk_downtown() -> Scene:
    roads = []
    for y in (35.0, 70.0, 105.0):                       # east-west streets
        roads.append((70.0, y, 70.0, 4.0))
    for x in (35.0, 70.0, 105.0):                       # north-south streets
        roads.append((x, 70.0, 4.0, 70.0))
    parked = [
        (18.0, 32.2, 2.2, 1.1), (44.0, 32.2, 2.2, 1.1), (78.0, 32.2, 2.2, 1.1),
        (112.0, 32.2, 2.2, 1.1), (24.0, 72.8, 2.2, 1.1), (96.0, 72.8, 2.2, 1.1),
        (124.0, 72.8, 2.2, 1.1), (58.0, 107.8, 2.2, 1.1), (92.0, 107.8, 2.2, 1.1),
        (32.2, 52.0, 1.1, 2.2), (72.8, 18.0, 1.1, 2.2), (107.8, 92.0, 1.1, 2.2),
    ]
    lights = [
        {"x": 35.0, "y": 70.0, "axis": "x", "dir": 1.0, "offset": 0.0},
        {"x": 70.0, "y": 70.0, "axis": "x", "dir": 1.0, "offset": 6.0},
        {"x": 105.0, "y": 70.0, "axis": "x", "dir": 1.0, "offset": 12.0},
    ]
    peds = [
        {"x0": 52.0, "y0": 64.0, "x1": 52.0, "y1": 76.0, "speed": 1.1, "phase": 0.0},
        {"x0": 88.0, "y0": 64.0, "x1": 88.0, "y1": 76.0, "speed": 1.3, "phase": 0.35},
    ]
    return Scene(
        name="downtown", width=140.0, height=140.0,
        start=(10.0, 70.0, 0.0, 0.0), goal=(130.0, 70.0, 0.0),
        roads=roads, parked=parked, lights=lights, pedestrians=peds,
        description="城市街区：三纵三横道路、三个信号灯路口、路边停车与两处行人横道，考察红绿灯通行、行人礼让与路口导航。",
        palette={"ground": (44, 62, 48), "road": (52, 56, 62), "lane": (232, 226, 200),
                 "curb": (150, 156, 150), "car": (70, 160, 220), "accent": (250, 200, 90)},
    )


def _mk_campus() -> Scene:
    roads = [
        (60.0, 82.0, 40.0, 4.0), (60.0, 18.0, 40.0, 4.0),
        (20.0, 50.0, 4.0, 32.0), (100.0, 50.0, 4.0, 32.0),
    ]
    trees = [(38.0, 38.0, 2.0), (62.0, 34.0, 1.6), (84.0, 62.0, 1.8),
             (44.0, 64.0, 1.5), (74.0, 46.0, 1.7)]
    barriers = [(60.0, 18.0, 3.0, 1.0)]
    peds = [
        {"x0": 60.0, "y0": 76.0, "x1": 60.0, "y1": 88.0, "speed": 1.2, "phase": 0.2},
        {"x0": 16.0, "y0": 50.0, "x1": 24.0, "y1": 50.0, "speed": 1.0, "phase": 0.6},
    ]
    return Scene(
        name="campus", width=120.0, height=100.0,
        start=(20.0, 50.0, 0.0, 1.5708), goal=(100.0, 50.0, 0.0),
        roads=roads, parked=[], cones=trees, barriers=barriers,
        lights=[], pedestrians=peds,
        description="园区环路：矩形环路、树木遮挡与两处行人横道，无信号灯，考察弯道跟踪与行人避让。",
        palette={"ground": (52, 72, 50), "road": (58, 60, 64), "lane": (226, 222, 200),
                 "curb": (156, 160, 150), "car": (240, 150, 60), "accent": (120, 220, 180)},
    )


def _mk_construction() -> Scene:
    roads = [(80.0, 50.0, 80.0, 5.0)]                    # one highway
    cones = []
    for i in range(9):                                   # lane-closure taper
        cones.append((66.0 + i * 2.2, 49.0 + i * 0.45, 0.45))
    barriers = [(76.0, 43.6, 5.0, 0.8), (86.0, 53.0, 4.0, 0.8)]
    parked = [(52.0, 46.6, 3.0, 1.2), (120.0, 53.2, 3.0, 1.2)]
    return Scene(
        name="construction", width=160.0, height=100.0,
        start=(10.0, 50.0, 0.0, 0.0), goal=(150.0, 50.0, 0.0),
        roads=roads, parked=parked, cones=cones, barriers=barriers,
        lights=[], pedestrians=[],
        description="施工路段：高速公路右车道封闭（锥桶渐变 + 护栏），需提前变道绕行，考察动态避障与狭窄通道通行。",
        palette={"ground": (62, 58, 46), "road": (54, 54, 58), "lane": (230, 222, 190),
                 "curb": (150, 146, 130), "car": (220, 90, 70), "accent": (250, 170, 80)},
    )


def get_scene(name: str) -> Scene:
    if name not in _SCENES:
        builders = {"downtown": _mk_downtown, "campus": _mk_campus,
                    "construction": _mk_construction}
        if name not in builders:
            raise KeyError(f"unknown scene {name!r}; available: {list(builders)}")
        _SCENES[name] = builders[name]()
    return _SCENES[name]


def scene_names() -> Sequence[str]:
    return tuple(C.SCENES)
