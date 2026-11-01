"""MuJoCo 水域场景与 ROV（遥控潜水器）模型。

坐标约定：x 前、y 左、z 上（米 / 弧度）。ROV 是自由刚体，
四个水平推进器 + 两个垂向推进器用 ``xfrc_applied`` 施加合力/合力矩，
重力由 MuJoCo 计算，浮力与阻力在 ``control.py`` 里做前馈补偿。

水体参数：密度 1000 kg/m³，ROV 排水体积 13.2 L（净浮力约 +3 N）。
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

MASS = 12.0                 # kg
VOLUME = 0.0132              # m³，排水体积
RHO = 1000.0                # kg/m³
GRAVITY = 9.81
BUOYANCY = RHO * 9.81 * VOLUME          # N，约 235 N（重力 117.7 N）
BODY_HALF = (0.32, 0.24, 0.18)
INERTIA = (0.45, 0.55, 0.65)

DRAG_LIN = (18.0, 22.0, 30.0)   # N·s/m，各轴线性阻力
DRAG_QUAD = (12.0, 15.0, 20.0)  # N·s²/m²

CRUISE_DEPTH = 1.0          # 默认定深，m（z 轴向上，水面 z=0）
SONAR_ALT_OFFSET = 0.0


@dataclass
class Obstacle:
    x: float
    y: float
    half_x: float
    half_y: float
    height: float = 2.0
    rgba: str = "0.35 0.45 0.40 1"


@dataclass
class SceneLayout:
    name: str
    size_x: float
    size_y: float
    start: Tuple[float, float, float]
    goal: Tuple[float, float]
    obstacles: List[Obstacle] = field(default_factory=list)
    seed: int = 0

    def as_dict(self) -> dict:
        return {"name": self.name, "size": [self.size_x, self.size_y],
                "start": list(self.start), "goal": list(self.goal),
                "obstacles": [{"x": o.x, "y": o.y, "hx": o.half_x, "hy": o.half_y,
                               "h": o.height} for o in self.obstacles]}


# ---------------------------------------------------------------------------
# MJCF
# ---------------------------------------------------------------------------
ROV_BODY = """
    <body name="rov" pos="{px:.4f} {py:.4f} {pz:.4f}"{extra}>
      <freejoint name="root"/>
      <geom name="hull" type="box" size="{hx} {hy} {hz}" mass="{mass}" rgba="0.85 0.55 0.15 1" group="3"/>
      <geom name="nose" type="box" pos="0.30 0 0" size="0.06 0.10 0.08" mass="0.4" rgba="0.9 0.75 0.2 1" group="3"/>
      <geom name="tank" type="cylinder" pos="0 0 0.16" size="0.10 0.10" mass="0.5" rgba="0.4 0.45 0.5 1" group="3"/>
      <site name="sonar" pos="0.42 0 0" size="0.02" rgba="0.2 0.9 0.9 1"/>
      <camera name="fpv" pos="0.40 0 0.05" xyaxes="0 1 0 0 0 1" fovy="80"/>
      <site name="rov" pos="0 0 0" size="0.05" rgba="0.9 0.3 0.3 1"/>
    </body>
"""


def rov_xml(px: float, py: float, pz: float, yaw: float = 0.0) -> str:
    extra = ""
    if yaw:
        extra = f' quat="{math.cos(yaw/2):.6f} 0 0 {math.sin(yaw/2):.6f}"'
    return ROV_BODY.format(px=px, py=py, pz=pz, extra=extra,
                           hx=BODY_HALF[0], hy=BODY_HALF[1], hz=BODY_HALF[2], mass=MASS)


def _obstacles(layout: SceneLayout) -> str:
    return "\n".join(
        f'    <geom name="obst_{i}" type="box" pos="{o.x:.4f} {o.y:.4f} {o.height/2:.4f}" '
        f'size="{o.half_x:.4f} {o.half_y:.4f} {o.height/2:.4f}" rgba="{o.rgba}"/>'
        for i, o in enumerate(layout.obstacles))


def build_xml(layout: SceneLayout, extra_xml: str = "") -> str:
    sx, sy = layout.size_x, layout.size_y
    walls = f"""
    <geom name="wall_n" type="box" pos="0 {sy/2:.3f} 1.1" size="{sx/2+0.2:.3f} 0.08 1.1" rgba="0.30 0.42 0.50 1"/>
    <geom name="wall_s" type="box" pos="0 {-sy/2:.3f} 1.1" size="{sx/2+0.2:.3f} 0.08 1.1" rgba="0.30 0.42 0.50 1"/>
    <geom name="wall_e" type="box" pos="{sx/2:.3f} 0 1.1" size="0.08 {sy/2:.3f} 1.1" rgba="0.26 0.38 0.46 1"/>
    <geom name="wall_w" type="box" pos="{-sx/2:.3f} 0 1.1" size="0.08 {sy/2:.3f} 1.1" rgba="0.26 0.38 0.46 1"/>
    """
    return f"""
<mujoco model="rov_mujoco_slam_nn_nav">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"
          gravity="0 0 -9.81"/>
  <visual>
    <global offwidth="960" offheight="720"/>
    <headlight diffuse="0.55 0.55 0.55" ambient="0.35 0.35 0.35" specular="0.10 0.10 0.10"/>
    <map znear="0.02" zfar="60"/>
    <quality shadowsize="2048"/>
  </visual>
  <worldbody>
    <camera name="follow_cam" pos="0 0 3" xyaxes="1 0 0 0 1 0" fovy="55"/>
    <light name="key" pos="2.0 -1.5 5.0" dir="-0.35 0.3 -1" directional="true"
           diffuse="0.55 0.55 0.55" specular="0.15 0.15 0.15"/>
    <geom name="floor" type="plane" size="{max(sx,sy)/2+2:.2f} {max(sx,sy)/2+2:.2f} 0.05"
          rgba="0.55 0.50 0.42 1" friction="1.0 0.02 0.002"/>
    <geom name="surface" type="plane" pos="0 0 4.0" size="{max(sx,sy)/2+2:.2f} {max(sx,sy)/2+2:.2f} 0.05"
          rgba="0.20 0.45 0.65 0.35" contype="0" conaffinity="0"/>
    <site name="goal" pos="{layout.goal[0]:.4f} {layout.goal[1]:.4f} 0.02" size="0.12"
          rgba="0.15 0.85 0.35 1"/>
{walls}
{_obstacles(layout)}
{rov_xml(layout.start[0], layout.start[1], CRUISE_DEPTH, layout.start[2])}
{extra_xml}
  </worldbody>
</mujoco>
"""


# ---------------------------------------------------------------------------
# 场景
# ---------------------------------------------------------------------------
def _pool(seed: int) -> SceneLayout:
    rng = random.Random(seed)
    obs = [
        Obstacle(0.4, 0.0, 0.28, 0.28, 2.0),
        Obstacle(1.6, 1.4, 0.25, 0.50, 2.0),
        Obstacle(1.6, -1.4, 0.25, 0.50, 2.0),
        Obstacle(2.8, 1.2, 0.40, 0.20, 2.0),
    ]
    for _ in range(2):
        obs.append(Obstacle(rng.uniform(-0.6, 2.2), rng.uniform(-1.4, 1.4), 0.13, 0.13, 1.8))
    return SceneLayout("pool", 7.0, 5.0, (-2.4, 0.0, 0.0), (2.6, 0.0), obs, seed)


def _pipe_field(seed: int) -> SceneLayout:
    rng = random.Random(seed + 17)
    obs = []
    for gx in (-0.9, 0.5, 1.9):
        for gy in (-1.2, 1.2):
            obs.append(Obstacle(gx + rng.uniform(-0.1, 0.1), gy + rng.uniform(-0.1, 0.1),
                                0.22, 0.22, 2.0))
    return SceneLayout("pipe_field", 7.0, 4.6, (-2.5, 0.0, 0.0), (2.5, 0.0), obs, seed)


def _trench(seed: int) -> SceneLayout:
    rng = random.Random(seed + 29)
    obs = []
    for i in range(4):
        x = -1.6 + i * 1.1
        side = 1 if i % 2 == 0 else -1
        obs.append(Obstacle(x, side * 1.10, 0.55, 0.10, 2.0))
        obs.append(Obstacle(x + 0.55, -side * 1.10, 0.55, 0.10, 2.0))
    obs.append(Obstacle(2.2, 1.0, 0.26, 0.26, 2.0))
    return SceneLayout("trench", 6.6, 3.6, (-2.4, 0.0, 0.0), (2.4, 0.0), obs, seed)


_SCENES = {"pool": _pool, "pipe_field": _pipe_field, "trench": _trench}


def make_layout(scene: str, seed: int = 0) -> SceneLayout:
    if scene not in _SCENES:
        raise KeyError(f"unknown scene {scene!r}, choose from {sorted(_SCENES)}")
    return _SCENES[scene](seed)


def scene_names() -> Sequence[str]:
    return tuple(_SCENES)
