"""MuJoCo 场景与四旋翼无人机模型构建。

用代码生成 MJCF：三个评测场景共用同一架无人机，只替换障碍物；障碍位置可按种子扰动。

坐标约定：x 前、y 左、z 上（米 / 弧度）。无人机是一个自由刚体，
四个旋翼用 ``xfrc_applied`` 施加合力/合力矩（见 ``flight.py``），
不依赖 MuJoCo 的旋翼插件，便于在纯 Python 里解释控制量。
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

# ---------------------------------------------------------------------------
# 机体参数
# ---------------------------------------------------------------------------
MASS = 1.2                 # kg
ARM = 0.22                 # 电机到机体中心距离，m
BODY_HALF = (0.16, 0.16, 0.05)
GRAVITY = 9.81
MAX_THRUST = 2.4 * MASS * GRAVITY     # N，约 2.4 倍重力

INERTIA = (0.012, 0.012, 0.020)       # 近似对角惯量，kg·m²

FLIGHT_ALT = 1.20          # 默认巡航高度，m
LIDAR_ALT_OFFSET = 0.0     # 激光装在机体中心高度


@dataclass
class Obstacle:
    x: float
    y: float
    half_x: float
    half_y: float
    height: float = 2.2
    rgba: str = "0.72 0.32 0.22 1"


@dataclass
class SceneLayout:
    name: str
    size_x: float
    size_y: float
    start: Tuple[float, float, float]     # x, y, yaw
    goal: Tuple[float, float]
    obstacles: List[Obstacle] = field(default_factory=list)
    seed: int = 0

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "size": [self.size_x, self.size_y],
            "start": list(self.start),
            "goal": list(self.goal),
            "obstacles": [
                {"x": o.x, "y": o.y, "hx": o.half_x, "hy": o.half_y, "h": o.height}
                for o in self.obstacles
            ],
        }


# ---------------------------------------------------------------------------
# MJCF
# ---------------------------------------------------------------------------
_ROTOR = """
      <geom type="cylinder" size="0.075 0.008" pos="{x:+.3f} {y:+.3f} 0.02"
            rgba="0.15 0.15 0.18 1" mass="0.02" group="3"/>
      <geom type="capsule" fromto="0 0 0 {x:+.3f} {y:+.3f} 0" size="0.012"
            rgba="0.30 0.30 0.34 1" mass="0.01" group="3"/>
"""

DRONE_BODY = """
    <body name="drone" pos="{px:.4f} {py:.4f} {pz:.4f}"{extra}>
      <freejoint name="root"/>
      <geom name="hull" type="box" size="{hx} {hy} {hz}" mass="{mass}" rgba="0.18 0.42 0.72 1" group="3"/>
      <geom name="nose" type="box" pos="0.15 0 0" size="0.05 0.06 0.03" mass="0.02" rgba="0.85 0.75 0.2 1" group="3"/>
{rotors}
      <site name="lidar" pos="0 0 0" size="0.02" rgba="0.2 0.9 0.9 1"/>
      <camera name="fpv" pos="0.12 0 0.02" xyaxes="0 1 0 0 0 1" fovy="80"/>
      <site name="drone" pos="0 0 0" size="0.05" rgba="0.9 0.3 0.3 1"/>
    </body>
"""


def drone_xml(px: float, py: float, pz: float, yaw: float = 0.0) -> str:
    rotors = "".join(
        _ROTOR.format(x=ARM * math.cos(a * math.pi / 2), y=ARM * math.sin(a * math.pi / 2))
        for a in range(4)
    )
    extra = ""
    if yaw:
        extra = f' quat="{math.cos(yaw/2):.6f} 0 0 {math.sin(yaw/2):.6f}"'
    return DRONE_BODY.format(px=px, py=py, pz=pz, extra=extra,
                             hx=BODY_HALF[0], hy=BODY_HALF[1], hz=BODY_HALF[2],
                             mass=MASS, rotors=rotors)


def _obstacles(layout: SceneLayout) -> str:
    parts = []
    for i, o in enumerate(layout.obstacles):
        parts.append(
            f'    <geom name="obst_{i}" type="box" pos="{o.x:.4f} {o.y:.4f} {o.height/2:.4f}" '
            f'size="{o.half_x:.4f} {o.half_y:.4f} {o.height/2:.4f}" rgba="{o.rgba}"/>'
        )
    return "\n".join(parts)


def build_xml(layout: SceneLayout, extra_xml: str = "") -> str:
    sx, sy = layout.size_x, layout.size_y
    walls = f"""
    <geom name="wall_n" type="box" pos="0 {sy/2:.3f} 1.25" size="{sx/2+0.2:.3f} 0.08 1.25" rgba="0.55 0.57 0.62 1"/>
    <geom name="wall_s" type="box" pos="0 {-sy/2:.3f} 1.25" size="{sx/2+0.2:.3f} 0.08 1.25" rgba="0.55 0.57 0.62 1"/>
    <geom name="wall_e" type="box" pos="{sx/2:.3f} 0 1.25" size="0.08 {sy/2:.3f} 1.25" rgba="0.48 0.50 0.55 1"/>
    <geom name="wall_w" type="box" pos="{-sx/2:.3f} 0 1.25" size="0.08 {sy/2:.3f} 1.25" rgba="0.48 0.50 0.55 1"/>
    """
    return f"""
<mujoco model="aerial_mujoco_slam_nn_nav">
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
          rgba="0.90 0.91 0.93 1" friction="1.2 0.02 0.002"/>
    <site name="goal" pos="{layout.goal[0]:.4f} {layout.goal[1]:.4f} 0.02" size="0.12"
          rgba="0.15 0.85 0.35 1"/>
{walls}
{_obstacles(layout)}
{drone_xml(layout.start[0], layout.start[1], FLIGHT_ALT, layout.start[2])}
{extra_xml}
  </worldbody>
</mujoco>
"""


# ---------------------------------------------------------------------------
# 场景
# ---------------------------------------------------------------------------
def _warehouse(seed: int) -> SceneLayout:
    rng = random.Random(seed)
    obs = [
        Obstacle(0.6, 0.0, 0.30, 0.30, 2.2),
        Obstacle(1.8, 1.5, 0.28, 0.55, 2.2),
        Obstacle(1.8, -1.5, 0.28, 0.55, 2.2),
        Obstacle(3.0, 0.3, 0.45, 0.22, 2.2),
    ]
    for _ in range(2):
        obs.append(Obstacle(rng.uniform(-0.6, 2.4), rng.uniform(-1.6, 1.6), 0.14, 0.14, 2.0))
    return SceneLayout("warehouse", 7.0, 5.0, (-2.4, 0.0, 0.0), (2.6, 0.0), obs, seed)


def _obstacle_field(seed: int) -> SceneLayout:
    rng = random.Random(seed + 11)
    obs = []
    for gx in (-0.9, 0.5, 1.9):
        for gy in (-1.2, 1.2):
            obs.append(Obstacle(gx + rng.uniform(-0.1, 0.1), gy + rng.uniform(-0.1, 0.1),
                                0.22, 0.22, 2.2))
    return SceneLayout("obstacle_field", 7.0, 4.6, (-2.5, 0.0, 0.0), (2.5, 0.0), obs, seed)


def _corridor(seed: int) -> SceneLayout:
    rng = random.Random(seed + 23)
    obs = []
    for i in range(3):
        x = -0.9 + i * 1.4
        side = 1 if i % 2 == 0 else -1
        obs.append(Obstacle(x, side * 1.15, 0.14, 0.45, 2.2))
        obs.append(Obstacle(x, -side * 1.15, 0.14, 0.45, 2.2))
    obs.append(Obstacle(2.3, rng.uniform(-0.2, 0.2), 0.26, 0.26, 2.2))
    return SceneLayout("corridor", 6.4, 3.4, (-2.2, 0.0, 0.0), (2.2, 0.0), obs, seed)


_SCENES = {
    "warehouse": _warehouse,
    "obstacle_field": _obstacle_field,
    "corridor": _corridor,
}


def make_layout(scene: str, seed: int = 0) -> SceneLayout:
    if scene not in _SCENES:
        raise KeyError(f"unknown scene {scene!r}, choose from {sorted(_SCENES)}")
    return _SCENES[scene](seed)


def scene_names() -> Sequence[str]:
    return tuple(_SCENES)
