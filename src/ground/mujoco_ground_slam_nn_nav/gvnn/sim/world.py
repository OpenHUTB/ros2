"""World definitions shared by the MuJoCo backend and the planners.

The world is a 24 m x 16 m indoor arena.  Three scenes exercise different
navigation skills:

* ``warehouse``   - parallel shelves, narrow aisles, a long detour;
* ``obstacle_field`` - scattered cylindrical obstacles in open space;
* ``crossing``    - a wall with a single 3 m gate, forcing a detour.

All geometry is expressed in metres in the ``map`` frame.  The same
description is rasterised into an occupancy grid and compiled into a MuJoCo
MJCF model, which guarantees that planning, SLAM and physics see exactly the
same world.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

import numpy as np

from ..config import MAP_RESOLUTION, WHEEL_RADIUS, WHEEL_TRACK

Point = Tuple[float, float]


@dataclass(frozen=True)
class Wall:
    """A thick line segment (the outer walls and thin partitions)."""

    x1: float
    y1: float
    x2: float
    y2: float
    thickness: float = 0.20
    height: float = 1.50
    color: Tuple[float, float, float] = (0.34, 0.38, 0.46)

    @property
    def length(self) -> float:
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)

    @property
    def angle(self) -> float:
        return math.atan2(self.y2 - self.y1, self.x2 - self.x1)

    @property
    def center(self) -> Point:
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)


@dataclass(frozen=True)
class CircleObstacle:
    x: float
    y: float
    r: float
    height: float = 1.0
    color: Tuple[float, float, float] = (0.92, 0.48, 0.15)


@dataclass(frozen=True)
class BlockObstacle:
    x: float
    y: float
    w: float
    h: float
    angle: float = 0.0
    height: float = 1.20
    color: Tuple[float, float, float] = (0.55, 0.60, 0.68)


@dataclass
class Scene:
    name: str
    description: str
    width: float
    height: float
    start: Point
    goal: Point
    start_yaw: float = 0.0
    walls: List[Wall] = field(default_factory=list)
    circles: List[CircleObstacle] = field(default_factory=list)
    blocks: List[BlockObstacle] = field(default_factory=list)

    # ------------------------------------------------------------------ utils
    def all_walls(self) -> List[Wall]:
        return list(self.walls)

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return margin <= x <= self.width - margin and margin <= y <= self.height - margin


# --------------------------------------------------------------------------- #
# Scene construction
# --------------------------------------------------------------------------- #
def _room(width: float = 24.0, height: float = 16.0, t: float = 0.20) -> List[Wall]:
    """Four walls whose inner faces bound [0, width] x [0, height]."""
    return [
        Wall(0.0, t / 2, width, t / 2, t),                 # south
        Wall(0.0, height - t / 2, width, height - t / 2, t),  # north
        Wall(t / 2, 0.0, t / 2, height, t),                 # west
        Wall(width - t / 2, 0.0, width - t / 2, height, t),  # east
    ]


def _warehouse() -> Scene:
    walls = _room()
    blocks: List[BlockObstacle] = []
    # Three double rows of shelves.  Shelves run 3 m in y with 4 m aisles.
    for x in (6.0, 12.0, 18.0):
        blocks.append(BlockObstacle(x, 4.5, 0.80, 3.0, 0.0, 1.10))
        blocks.append(BlockObstacle(x, 11.5, 0.80, 3.0, 0.0, 1.10))
    # Pallets make the central aisle non-trivial but keep a 1.5 m corridor.
    circles = [
        CircleObstacle(4.0, 8.0, 0.45, 0.75),
        CircleObstacle(9.6, 7.2, 0.40, 0.65),
        CircleObstacle(15.2, 9.0, 0.45, 0.75),
        CircleObstacle(20.6, 3.0, 0.40, 0.65),
    ]
    return Scene(
        name="warehouse",
        description="仓库货架场景：三条双排货架、窄巷道与托盘障碍",
        width=24.0,
        height=16.0,
        start=(2.0, 2.0),
        goal=(22.0, 14.0),
        walls=walls,
        circles=circles,
        blocks=blocks,
    )


def _obstacle_field() -> Scene:
    walls = _room()
    circles = [
        CircleObstacle(5.0, 4.0, 0.60),
        CircleObstacle(8.0, 8.5, 0.70),
        CircleObstacle(11.0, 4.5, 0.50),
        CircleObstacle(13.5, 10.5, 0.80),
        CircleObstacle(17.0, 5.5, 0.60),
        CircleObstacle(20.0, 11.0, 0.70),
        CircleObstacle(7.0, 13.0, 0.50),
        CircleObstacle(12.5, 13.8, 0.55),
        CircleObstacle(18.0, 14.0, 0.50),
        CircleObstacle(3.5, 10.0, 0.50),
    ]
    blocks = [
        BlockObstacle(9.5, 11.0, 1.2, 0.5, 0.35, 1.0),
        BlockObstacle(15.5, 2.8, 1.0, 0.5, -0.40, 1.0),
    ]
    return Scene(
        name="obstacle_field",
        description="随机障碍场景：圆柱与斜置货箱，验证局部避障",
        width=24.0,
        height=16.0,
        start=(2.0, 2.0),
        goal=(22.0, 14.0),
        walls=walls,
        circles=circles,
        blocks=blocks,
    )


def _crossing() -> Scene:
    walls = _room()
    # A full-width wall with one 3.0 m gate at x in [10.5, 13.5], y = 8.
    walls.append(Wall(0.0, 8.0, 10.5, 8.0, 0.25))
    walls.append(Wall(13.5, 8.0, 24.0, 8.0, 0.25))
    circles = [
        CircleObstacle(8.6, 8.0, 0.55),
        CircleObstacle(15.4, 8.0, 0.55),
        CircleObstacle(12.0, 5.4, 0.45),
        CircleObstacle(19.5, 12.0, 0.50),
    ]
    blocks = [
        BlockObstacle(4.0, 12.0, 2.0, 0.6, 0.0, 1.0),
        BlockObstacle(19.0, 3.0, 2.0, 0.6, 0.0, 1.0),
    ]
    return Scene(
        name="crossing",
        description="单门通道场景：整墙仅留 3 m 门洞，验证全局绕行",
        width=24.0,
        height=16.0,
        start=(2.0, 2.0),
        goal=(22.0, 14.0),
        walls=walls,
        circles=circles,
        blocks=blocks,
    )


SCENES: Dict[str, Scene] = {
    "warehouse": _warehouse(),
    "obstacle_field": _obstacle_field(),
    "crossing": _crossing(),
}


def get_scene(name: str) -> Scene:
    if name not in SCENES:
        raise KeyError(f"unknown scene {name!r}; choose from {sorted(SCENES)}")
    return SCENES[name]


def scene_names() -> List[str]:
    return list(SCENES.keys())


# --------------------------------------------------------------------------- #
# Rasterisation
# --------------------------------------------------------------------------- #
def occupancy(scene: Scene, resolution: float = MAP_RESOLUTION) -> np.ndarray:
    """Boolean occupancy grid, shape (H, W), row-major with origin at (0, 0)."""
    w = int(round(scene.width / resolution))
    h = int(round(scene.height / resolution))
    grid = np.zeros((h, w), dtype=bool)

    for wall in scene.walls:
        half = wall.thickness / 2.0
        xmin = max(0, int((min(wall.x1, wall.x2) - half) / resolution) - 1)
        xmax = min(w - 1, int((max(wall.x1, wall.x2) + half) / resolution) + 1)
        ymin = max(0, int((min(wall.y1, wall.y2) - half) / resolution) - 1)
        ymax = min(h - 1, int((max(wall.y1, wall.y2) + half) / resolution) + 1)
        for gy in range(ymin, ymax + 1):
            py = (gy + 0.5) * resolution
            for gx in range(xmin, xmax + 1):
                px = (gx + 0.5) * resolution
                vx, vy = wall.x2 - wall.x1, wall.y2 - wall.y1
                wx, wy = px - wall.x1, py - wall.y1
                denom = vx * vx + vy * vy
                t = 0.0 if denom < 1e-12 else max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
                d = math.hypot(px - (wall.x1 + t * vx), py - (wall.y1 + t * vy))
                if d <= half:
                    grid[gy, gx] = True

    for block in scene.blocks:
        c, s = math.cos(block.angle), math.sin(block.angle)
        xmin = max(0, int((block.x - block.w - block.h) / resolution) - 1)
        xmax = min(w - 1, int((block.x + block.w + block.h) / resolution) + 1)
        ymin = max(0, int((block.y - block.w - block.h) / resolution) - 1)
        ymax = min(h - 1, int((block.y + block.w + block.h) / resolution) + 1)
        for gy in range(ymin, ymax + 1):
            py = (gy + 0.5) * resolution - block.y
            for gx in range(xmin, xmax + 1):
                px = (gx + 0.5) * resolution - block.x
                lx = c * px + s * py
                ly = -s * px + c * py
                if abs(lx) <= block.w / 2.0 and abs(ly) <= block.h / 2.0:
                    grid[gy, gx] = True

    for ob in scene.circles:
        xmin = max(0, int((ob.x - ob.r) / resolution) - 1)
        xmax = min(w - 1, int((ob.x + ob.r) / resolution) + 1)
        ymin = max(0, int((ob.y - ob.r) / resolution) - 1)
        ymax = min(h - 1, int((ob.y + ob.r) / resolution) + 1)
        for gy in range(ymin, ymax + 1):
            py = (gy + 0.5) * resolution
            for gx in range(xmin, xmax + 1):
                px = (gx + 0.5) * resolution
                if (px - ob.x) ** 2 + (py - ob.y) ** 2 <= ob.r ** 2:
                    grid[gy, gx] = True
    return grid


def scene_rgb(scene: Scene, resolution: float = MAP_RESOLUTION) -> np.ndarray:
    """Colour raster of the world used by the bird's-eye camera and renderer."""
    w = int(round(scene.width / resolution))
    h = int(round(scene.height / resolution))
    img = np.empty((h, w, 3), dtype=np.float32)
    img[:] = np.array([0.20, 0.23, 0.26], dtype=np.float32)  # floor

    def put_circle(cx: float, cy: float, r: float, color: Sequence[float]) -> None:
        xmin = max(0, int((cx - r) / resolution) - 1)
        xmax = min(w - 1, int((cx + r) / resolution) + 1)
        ymin = max(0, int((cy - r) / resolution) - 1)
        ymax = min(h - 1, int((cy + r) / resolution) + 1)
        for gy in range(ymin, ymax + 1):
            py = (gy + 0.5) * resolution
            for gx in range(xmin, xmax + 1):
                px = (gx + 0.5) * resolution
                if (px - cx) ** 2 + (py - cy) ** 2 <= r ** 2:
                    img[gy, gx] = color

    for wall in scene.walls:
        half = wall.thickness / 2.0
        xmin = max(0, int((min(wall.x1, wall.x2) - half) / resolution) - 1)
        xmax = min(w - 1, int((max(wall.x1, wall.x2) + half) / resolution) + 1)
        ymin = max(0, int((min(wall.y1, wall.y2) - half) / resolution) - 1)
        ymax = min(h - 1, int((max(wall.y1, wall.y2) + half) / resolution) + 1)
        for gy in range(ymin, ymax + 1):
            py = (gy + 0.5) * resolution
            for gx in range(xmin, xmax + 1):
                px = (gx + 0.5) * resolution
                vx, vy = wall.x2 - wall.x1, wall.y2 - wall.y1
                wx, wy = px - wall.x1, py - wall.y1
                denom = vx * vx + vy * vy
                t = 0.0 if denom < 1e-12 else max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
                if math.hypot(px - (wall.x1 + t * vx), py - (wall.y1 + t * vy)) <= half:
                    img[gy, gx] = wall.color

    for block in scene.blocks:
        c, s = math.cos(block.angle), math.sin(block.angle)
        xmin = max(0, int((block.x - block.w - block.h) / resolution) - 1)
        xmax = min(w - 1, int((block.x + block.w + block.h) / resolution) + 1)
        ymin = max(0, int((block.y - block.w - block.h) / resolution) - 1)
        ymax = min(h - 1, int((block.y + block.w + block.h) / resolution) + 1)
        for gy in range(ymin, ymax + 1):
            py = (gy + 0.5) * resolution - block.y
            for gx in range(xmin, xmax + 1):
                px = (gx + 0.5) * resolution - block.x
                lx = c * px + s * py
                ly = -s * px + c * py
                if abs(lx) <= block.w / 2.0 and abs(ly) <= block.h / 2.0:
                    img[gy, gx] = block.color

    for ob in scene.circles:
        put_circle(ob.x, ob.y, ob.r, ob.color)
    put_circle(scene.goal[0], scene.goal[1], 0.40, (0.10, 0.85, 0.30))
    return np.clip(img, 0.0, 1.0)


def free_mask(scene: Scene, resolution: float = MAP_RESOLUTION,
              clearance: float = 0.0) -> np.ndarray:
    from ..geometry import inflate

    occ = occupancy(scene, resolution)
    if clearance <= 0:
        return ~occ
    return ~inflate(occ, int(round(clearance / resolution)))


def world_to_cell(x: float, y: float, resolution: float = MAP_RESOLUTION) -> Tuple[int, int]:
    return int(x / resolution), int(y / resolution)


def cell_to_world(cx: int, cy: int, resolution: float = MAP_RESOLUTION) -> Point:
    return (cx + 0.5) * resolution, (cy + 0.5) * resolution


# --------------------------------------------------------------------------- #
# MuJoCo model
# --------------------------------------------------------------------------- #
def _fmt(values: Sequence[float]) -> str:
    return " ".join(f"{v:.5f}" for v in values)


def mujoco_xml(scene: Scene, robot: bool = True) -> str:
    """Compile the scene into a MuJoCo MJCF string.

    Geometry groups: 0 = static structure, 1 = obstacles, 2 = non-colliding
    goal marker, 3 = robot.  The lidar ray mask is ``[1, 1, 0, 0, 0, 0]`` so
    rays see the world but not the robot itself.
    """
    geoms: List[str] = [
        '<geom name="floor" type="plane" size="40 40 .1" rgba="0.30 0.33 0.36 1" group="0"/>'
    ]
    for i, wall in enumerate(scene.walls):
        cx, cy = wall.center
        geoms.append(
            f'<geom name="wall{i}" type="box" pos="{cx:.4f} {cy:.4f} {wall.height / 2:.4f}" '
            f'size="{wall.length / 2:.4f} {wall.thickness / 2:.4f} {wall.height / 2:.4f}" '
            f'euler="0 0 {wall.angle:.5f}" rgba="{wall.color[0]} {wall.color[1]} {wall.color[2]} 1" group="0"/>'
        )
    for i, ob in enumerate(scene.circles):
        geoms.append(
            f'<geom name="obs{i}" type="cylinder" pos="{ob.x:.4f} {ob.y:.4f} {ob.height / 2:.4f}" '
            f'size="{ob.r:.4f} {ob.height / 2:.4f}" rgba="{ob.color[0]} {ob.color[1]} {ob.color[2]} 1" group="1"/>'
        )
    for i, block in enumerate(scene.blocks):
        geoms.append(
            f'<geom name="box{i}" type="box" pos="{block.x:.4f} {block.y:.4f} {block.height / 2:.4f}" '
            f'size="{block.w / 2:.4f} {block.h / 2:.4f} {block.height / 2:.4f}" '
            f'euler="0 0 {block.angle:.5f}" rgba="{block.color[0]} {block.color[1]} {block.color[2]} 1" group="1"/>'
        )
    geoms.append(
        f'<geom name="goal" type="cylinder" pos="{scene.goal[0]:.4f} {scene.goal[1]:.4f} 0.012" '
        f'size="0.40 0.012" rgba="0.10 0.85 0.30 1" group="2" contype="0" conaffinity="0"/>'
    )
    if robot:
        qw, qz = math.cos(scene.start_yaw / 2.0), math.sin(scene.start_yaw / 2.0)
        geoms.append(_robot_xml(scene.start[0], scene.start[1], qw, qz))
    return f"""<mujoco model="gvnn_{scene.name}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>
  <default>
    <geom friction="1.0 0.005 0.0001" condim="3" solref="0.004 1" solimp="0.9 0.95 0.001"/>
    <joint damping="0.0005"/>
    <motor ctrlrange="-1 1"/>
  </default>
  <worldbody>
    {chr(10).join('    ' + g for g in geoms)}
  </worldbody>
  <actuator>
    <velocity name="left_motor" joint="left" kv="0.8" ctrllimited="true" ctrlrange="-25 25" forcelimited="true" forcerange="-3 3"/>
    <velocity name="right_motor" joint="right" kv="0.8" ctrllimited="true" ctrlrange="-25 25" forcelimited="true" forcerange="-3 3"/>
  </actuator>
</mujoco>"""


def _robot_xml(x: float, y: float, qw: float, qz: float) -> str:
    z = 0.13
    return f"""<body name="chassis" pos="{x:.5f} {y:.5f} {z}" quat="{qw:.6f} 0 0 {qz:.6f}">
      <freejoint name="free"/>
      <geom name="body" type="box" size="0.28 0.18 0.06" mass="4.0" rgba="0.12 0.62 0.90 1" group="3"/>
      <geom name="nose" type="box" pos="0.24 0 0.08" size="0.05 0.06 0.02" mass="0.03" rgba="1.0 0.85 0.15 1" group="3"/>
      <site name="lidar_site" pos="0 0 0.12" size="0.02" rgba="1 0 0 1" group="3"/>
      <site name="imu_site" pos="0 0 0" size="0.01" rgba="0 1 0 1" group="3"/>
      <body name="left_wheel" pos="-0.08 0.22 -0.03">
        <joint name="left" type="hinge" axis="0 1 0"/>
        <geom type="ellipsoid" size="0.10 0.04 0.10" mass="0.28" rgba="0.07 0.08 0.09 1" group="3"/>
      </body>
      <body name="right_wheel" pos="-0.08 -0.22 -0.03">
        <joint name="right" type="hinge" axis="0 1 0"/>
        <geom type="ellipsoid" size="0.10 0.04 0.10" mass="0.28" rgba="0.07 0.08 0.09 1" group="3"/>
      </body>
      <geom name="caster" type="sphere" pos="0.22 0 -0.085" size="0.045" mass="0.08"
            priority="1" condim="1" rgba="0.1 0.1 0.1 1" group="3"/>
    </body>"""
