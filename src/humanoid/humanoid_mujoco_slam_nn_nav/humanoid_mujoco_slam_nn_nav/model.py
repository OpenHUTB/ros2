"""MuJoCo 场景与双足人形机器人模型构建。

本模块用代码生成 MJCF（而不是维护一份巨大的静态 XML），这样：
* 三个评测场景（仓库 / 障碍场 / 门洞走廊）共享同一套人体结构，只替换障碍物；
* 障碍物位置可按随机种子扰动，便于做多场景统计；
* 规划器与评价脚本可以直接读取 ``SceneLayout``，不必反解析 XML。

坐标约定：x 前、y 左、z 上（米 / 弧度）。
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

# ---------------------------------------------------------------------------
# 人体几何参数（单位 m / kg）。IK 与 MJCF 共用同一组常量，避免两处写死不一致。
# ---------------------------------------------------------------------------
PELVIS_HEIGHT = 0.80          # 站立时骨盆中心高度
HIP_OFFSET_Y = 0.095          # 髋关节相对骨盆中心的横向偏移（单侧）
HIP_OFFSET_Z = -0.03          # 髋关节相对骨盆中心的高度偏移
THIGH_LEN = 0.34              # 大腿长度（髋 -> 膝）
SHIN_LEN = 0.34               # 小腿长度（膝 -> 踝）
ANKLE_TO_SOLE = 0.085         # 踝关节中心到脚底的距离
FOOT_HALF_LEN = 0.15          # 脚掌半长
FOOT_HALF_WID = 0.080         # 脚掌半宽（略宽于真人，提高侧向稳定裕度）
STAND_ANKLE_Z = 0.105         # 站立相踝关节目标高度

LEG_JOINTS: Tuple[str, ...] = (
    "hip_yaw", "hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll",
)
ACTUATED_JOINTS: Tuple[str, ...] = tuple(
    f"{j}_{s}" for s in ("l", "r") for j in LEG_JOINTS
) + ("waist_yaw", "shoulder_pitch_l", "shoulder_pitch_r",
     "elbow_pitch_l", "elbow_pitch_r")


@dataclass
class Obstacle:
    """轴对齐长方体障碍物（规划器与仿真共用）。"""

    x: float
    y: float
    half_x: float
    half_y: float
    height: float = 0.9
    rgba: str = "0.75 0.25 0.20 1"

    @property
    def bounds(self) -> Tuple[float, float, float, float]:
        return (self.x - self.half_x, self.x + self.half_x,
                self.y - self.half_y, self.y + self.half_y)


@dataclass
class SceneLayout:
    """一个场景的完整描述：起点、终点、障碍物、场地边界。"""

    name: str
    size_x: float
    size_y: float
    start: Tuple[float, float, float]        # x, y, yaw
    goal: Tuple[float, float]                # x, y
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
# MJCF 片段
# ---------------------------------------------------------------------------
_LEG_TEMPLATE = """
      <body name="{side}hip" pos="0 {hy:+.4f} {hz:+.4f}">
        <joint name="hip_yaw_{s}" axis="0 0 1" range="-0.8 0.8" damping="0.8" armature="0.03"/>
        <joint name="hip_roll_{s}" axis="1 0 0" range="-0.9 0.9" damping="0.6" armature="0.02"/>
        <geom type="sphere" size="0.055" mass="0.4" rgba="0.22 0.24 0.30 1" group="3" contype="0" conaffinity="0"/>
        <body name="{side}thigh" pos="0 0 0">
          <joint name="hip_pitch_{s}" axis="0 1 0" range="-1.9 1.2" damping="0.6" armature="0.02"/>
          <geom type="capsule" fromto="0 0 -0.02 0 0 -{thigh:.4f}" size="0.062" mass="{m_thigh}" group="3" contype="0" conaffinity="0"/>
          <body name="{side}shin" pos="0 0 -{thigh:.4f}">
            <joint name="knee_pitch_{s}" axis="0 1 0" range="-0.05 2.1" damping="0.5" armature="0.02"/>
            <geom type="capsule" fromto="0 0 -0.02 0 0 -{shin:.4f}" size="0.052" mass="{m_shin}" group="3" contype="0" conaffinity="0"/>
            <body name="{side}foot" pos="0 0 -{shin:.4f}">
              <joint name="ankle_pitch_{s}" axis="0 1 0" range="-1.1 1.1" damping="0.35" armature="0.01"/>
              <joint name="ankle_roll_{s}" axis="1 0 0" range="-0.8 0.8" damping="0.35" armature="0.01"/>
              <geom name="{side}_sole" type="box" pos="0.035 0 -{sole:.4f}"
                    size="{flen:.4f} {fwid:.4f} 0.030" mass="{m_foot}"
                    rgba="0.15 0.16 0.20 1" group="3" friction="1.4 0.03 0.002"/>
              <site name="{side}_ankle" pos="0 0 0" size="0.02" rgba="0.9 0.6 0.1 1"/>
            </body>
          </body>
        </body>
      </body>
"""

_ARM_TEMPLATE = """
      <body name="{side}shoulder" pos="0.0 {ay:+.4f} 0.235">
        <joint name="shoulder_pitch_{s}" axis="0 1 0" range="-1.4 1.4" damping="0.4" armature="0.01"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.26" size="0.042" mass="1.2" rgba="0.24 0.26 0.32 1" group="3" contype="0" conaffinity="0"/>
        <body name="{side}forearm" pos="0 0 -0.26">
          <joint name="elbow_pitch_{s}" axis="0 1 0" range="-1.9 0.1" damping="0.3" armature="0.01"/>
          <geom type="capsule" fromto="0 0 0 0 0 -0.24" size="0.036" mass="0.9" rgba="0.28 0.30 0.36 1" group="3" contype="0" conaffinity="0"/>
        </body>
      </body>
"""

ROBOT_BODY = """
    <body name="pelvis" pos="{px:.4f} {py:.4f} {pz:.4f}"{extra}>
      <freejoint name="root"/>
      <geom type="box" size="0.125 0.105 0.075" mass="11.0" rgba="0.20 0.45 0.72 1" group="3" contype="0" conaffinity="0"/>
      <body name="torso" pos="0 0 0.075">
        <joint name="waist_yaw" axis="0 0 1" range="-1.2 1.2" damping="1.0" armature="0.02"/>
        <geom type="box" size="0.115 0.095 0.155" pos="0 0 0.155" mass="9.0" rgba="0.22 0.50 0.78 1" group="3" contype="0" conaffinity="0"/>
        <geom type="sphere" pos="0.005 0 0.375" size="0.095" mass="1.6" rgba="0.85 0.72 0.55 1" group="3" contype="0" conaffinity="0"/>
        <site name="lidar" pos="0 0 -0.18" size="0.02" rgba="0.2 0.9 0.9 1"/>
        <camera name="head_cam" pos="0.135 0 0.400" xyaxes="0 1 0 0 0 1" fovy="70"/>
{arms}
      </body>
{legs}
    </body>
"""


def _yaw_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def robot_body_xml(px: float = 0.0, py: float = 0.0, pz: float = PELVIS_HEIGHT,
                   yaw: float = 0.0) -> str:
    legs = "".join(
        _LEG_TEMPLATE.format(
            side=s, s=s,
            hy=HIP_OFFSET_Y if s == "l" else -HIP_OFFSET_Y,
            hz=HIP_OFFSET_Z,
            thigh=THIGH_LEN, shin=SHIN_LEN, sole=ANKLE_TO_SOLE,
            flen=FOOT_HALF_LEN, fwid=FOOT_HALF_WID,
            m_thigh=3.4, m_shin=2.4, m_foot=0.9,
        )
        for s in ("l", "r")
    )
    arms = "".join(
        _ARM_TEMPLATE.format(side=a, s=a, ay=0.155 if a == "l" else -0.155)
        for a in ("l", "r")
    )
    extra = ""
    if yaw:
        q = _yaw_quat(yaw)
        extra = f' quat="{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}"'
    return ROBOT_BODY.format(px=px, py=py, pz=pz, legs=legs, arms=arms, extra=extra)


def _actuators() -> str:
    rows = []
    for s in ("l", "r"):
        rows += [
            f'    <position name="a_hip_yaw_{s}" joint="hip_yaw_{s}" kp="420" kv="24" forcerange="-160 160"/>',
            f'    <position name="a_hip_roll_{s}" joint="hip_roll_{s}" kp="950" kv="45" forcerange="-380 380"/>',
            f'    <position name="a_hip_pitch_{s}" joint="hip_pitch_{s}" kp="600" kv="30" forcerange="-320 320"/>',
            f'    <position name="a_knee_pitch_{s}" joint="knee_pitch_{s}" kp="600" kv="30" forcerange="-320 320"/>',
            f'    <position name="a_ankle_pitch_{s}" joint="ankle_pitch_{s}" kp="700" kv="32" forcerange="-260 260"/>',
            f'    <position name="a_ankle_roll_{s}" joint="ankle_roll_{s}" kp="700" kv="32" forcerange="-260 260"/>',
        ]
    rows += [
        '<position name="a_waist_yaw" joint="waist_yaw" kp="220" kv="12" forcerange="-120 120"/>',
        '<position name="a_shoulder_pitch_l" joint="shoulder_pitch_l" kp="120" kv="8" forcerange="-90 90"/>',
        '<position name="a_shoulder_pitch_r" joint="shoulder_pitch_r" kp="120" kv="8" forcerange="-90 90"/>',
        '<position name="a_elbow_pitch_l" joint="elbow_pitch_l" kp="90" kv="6" forcerange="-60 60"/>',
        '<position name="a_elbow_pitch_r" joint="elbow_pitch_r" kp="90" kv="6" forcerange="-60 60"/>',
    ]
    return "\n".join(rows)


def _scene_obstacles(layout: SceneLayout) -> str:
    parts = []
    for i, o in enumerate(layout.obstacles):
        parts.append(
            f'    <geom name="obst_{i}" type="box" pos="{o.x:.4f} {o.y:.4f} {o.height / 2:.4f}" '
            f'size="{o.half_x:.4f} {o.half_y:.4f} {o.height / 2:.4f}" rgba="{o.rgba}"/>'
        )
    return "\n".join(parts)


def build_xml(layout: SceneLayout, extra_xml: str = "") -> str:
    """生成可直接喂给 ``mujoco.MjModel.from_xml_string`` 的完整模型。"""
    sx, sy = layout.size_x, layout.size_y
    walls = f"""
    <geom name="wall_n" type="box" pos="0 {sy / 2:.3f} 0.6" size="{sx / 2 + 0.2:.3f} 0.1 0.6" rgba="0.55 0.57 0.62 1"/>
    <geom name="wall_s" type="box" pos="0 {-sy / 2:.3f} 0.6" size="{sx / 2 + 0.2:.3f} 0.1 0.6" rgba="0.55 0.57 0.62 1"/>
    <geom name="wall_e" type="box" pos="{sx / 2:.3f} 0 0.6" size="0.1 {sy / 2:.3f} 0.6" rgba="0.48 0.50 0.55 1"/>
    <geom name="wall_w" type="box" pos="{-sx / 2:.3f} 0 0.6" size="0.1 {sy / 2:.3f} 0.6" rgba="0.48 0.50 0.55 1"/>
    """
    return f"""
<mujoco model="humanoid_mujoco_slam_nn_nav">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"
          gravity="0 0 -9.81"/>
  <visual>
    <global offwidth="960" offheight="720"/>
    <headlight diffuse="0.55 0.55 0.55" ambient="0.30 0.30 0.30" specular="0.10 0.10 0.10"/>
    <map znear="0.02" zfar="60"/>
    <quality shadowsize="2048"/>
  </visual>
  <worldbody>
    <camera name="follow_cam" pos="0 0 3" xyaxes="1 0 0 0 1 0" fovy="55"/>
    <light name="key" pos="2.0 -1.5 4.0" dir="-0.35 0.3 -1" directional="true"
           diffuse="0.55 0.55 0.55" specular="0.15 0.15 0.15"/>
    <geom name="floor" type="plane" size="{max(sx, sy) / 2 + 2:.2f} {max(sx, sy) / 2 + 2:.2f} 0.05"
          rgba="0.90 0.91 0.93 1" friction="1.4 0.03 0.002"/>
    <site name="goal" pos="{layout.goal[0]:.4f} {layout.goal[1]:.4f} 0.02" size="0.12"
          rgba="0.15 0.85 0.35 1"/>
{walls}
{_scene_obstacles(layout)}
{robot_body_xml(layout.start[0], layout.start[1], PELVIS_HEIGHT, layout.start[2])}
{extra_xml}
  </worldbody>
  <actuator>
{_actuators()}
  </actuator>
</mujoco>
"""


# ---------------------------------------------------------------------------
# 场景定义
# ---------------------------------------------------------------------------
def _warehouse(seed: int) -> SceneLayout:
    rng = random.Random(seed)
    obs = [
        Obstacle(2.2, 0.0, 0.35, 0.35, 0.9),
        Obstacle(3.4, 1.6, 0.30, 0.60, 0.9),
        Obstacle(3.4, -1.6, 0.30, 0.60, 0.9),
        Obstacle(5.0, 0.4, 0.55, 0.28, 0.9),
    ]
    for _ in range(3):
        obs.append(Obstacle(1.2 + rng.uniform(0, 4.6), rng.uniform(-1.6, 1.6),
                            0.16, 0.16, 0.8))
    return SceneLayout("warehouse", 8.0, 5.0, (-1.8, 0.0, 0.0), (2.4, 0.0), obs, seed)


def _obstacle_field(seed: int) -> SceneLayout:
    rng = random.Random(seed + 101)
    obs = []
    # 两排错位障碍，中间留出约 0.9 m 的可行走通道（双足膨胀半径 ~0.35 m）
    for gx in (-1.0, 0.6):
        for gy in (-1.05, 1.05):
            obs.append(Obstacle(gx + rng.uniform(-0.12, 0.12),
                                gy + rng.uniform(-0.12, 0.12),
                                0.22, 0.22, 0.9))
    return SceneLayout("obstacle_field", 8.0, 5.0, (-1.9, 0.0, 0.0), (2.0, 0.0), obs, seed)


def _corridor(seed: int) -> SceneLayout:
    rng = random.Random(seed + 202)
    obs = []
    for i in range(4):
        x = -1.6 + i * 1.1
        side = 1 if i % 2 == 0 else -1
        obs.append(Obstacle(x, side * 0.95, 0.55, 0.10, 0.9))
        obs.append(Obstacle(x + 0.55, -side * 0.95, 0.55, 0.10, 0.9))
    obs.append(Obstacle(2.4, rng.uniform(-0.2, 0.2), 0.28, 0.28, 0.9))
    return SceneLayout("corridor", 7.0, 3.6, (-1.7, 0.0, 0.0), (1.8, 0.0), obs, seed)


_SCENE_FACTORIES = {
    "warehouse": _warehouse,
    "obstacle_field": _obstacle_field,
    "corridor": _corridor,
}


def make_layout(scene: str, seed: int = 0) -> SceneLayout:
    if scene not in _SCENE_FACTORIES:
        raise KeyError(f"unknown scene {scene!r}, choose from {sorted(_SCENE_FACTORIES)}")
    return _SCENE_FACTORIES[scene](seed)


def scene_names() -> Sequence[str]:
    return tuple(_SCENE_FACTORIES)
