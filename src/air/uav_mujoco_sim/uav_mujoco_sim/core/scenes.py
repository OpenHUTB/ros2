#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场景库：以 ``models/quadrotor.xml`` 为模板，程序化注入场景几何。

场景 geom 追加到同一 worldbody（不用 ``<include>``，会产生嵌套 worldbody）；
geom 名字前缀 floor_/wall_/tree_/obstacle_/gate_/car_ 决定语义类别。
"""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    from .paths import models_dir as _models_dir
    MODEL_DIR = _models_dir()
except Exception:                                    # noqa: BLE001
    _PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    MODEL_DIR = os.path.join(os.path.dirname(_PKG_ROOT), "models")
    if not os.path.isdir(MODEL_DIR):
        MODEL_DIR = os.path.join(_PKG_ROOT, "models")
QUADROTOR_XML = os.path.join(MODEL_DIR, "quadrotor.xml")


# --------------------------------------------------------------------------- #
# 场景基元
# --------------------------------------------------------------------------- #
@dataclass
class SceneObject:
    """场景几何体；name 前缀决定其在语义分割里的类别。"""

    name: str
    kind: str                       # box / cylinder / sphere / capsule
    pos: Tuple[float, float, float]
    size: Tuple[float, ...]
    rgba: Tuple[float, float, float, float] = (0.5, 0.5, 0.5, 1.0)
    quat: Optional[Tuple[float, float, float, float]] = None

    def to_xml(self) -> str:
        attrs = [f'name="{self.name}"', f'type="{self.kind}"',
                 f'pos="{self.pos[0]:.4f} {self.pos[1]:.4f} {self.pos[2]:.4f}"',
                 f'size="{" ".join(f"{s:.4f}" for s in self.size)}"',
                 f'rgba="{" ".join(f"{c:.3f}" for c in self.rgba)}"']
        if self.quat is not None:
            attrs.append(f'quat="{self.quat[0]:.5f} {self.quat[1]:.5f} '
                         f'{self.quat[2]:.5f} {self.quat[3]:.5f}"')
        return "      <geom " + " ".join(attrs) + "/>"


@dataclass
class Scene:
    """一个完整场景：地面 + 障碍列表 + 附加配置。"""

    name: str
    ground_size: Tuple[float, float] = (60.0, 60.0)
    objects: List[SceneObject] = field(default_factory=list)
    # 无人机初始位置与朝向
    drone_pos: Tuple[float, float, float] = (0.0, 0.0, 1.5)
    drone_yaw: float = 0.0
    # 覆盖 option 的 timestep；None 用模板值
    timestep: Optional[float] = None
    notes: str = ""


# --------------------------------------------------------------------------- #
# 场景构造器
# --------------------------------------------------------------------------- #
def open_ground(seed: int = 0) -> Scene:
    """开阔平地：用于控制类任务（轨迹跟踪、抗风扰）的干净背景。"""
    return Scene(name="open_ground", ground_size=(60.0, 60.0),
                 notes="开阔平地，无遮挡。用于控制类实验与悬停/轨迹跟踪。")


def obstacle_field(seed: int = 0, n_obstacles: int = 20,
                   area: float = 10.0) -> Scene:
    """随机障碍场：立方体/圆柱/球混合，用于避障、导航与安全过滤器。"""
    rng = np.random.default_rng(seed)
    objs: List[SceneObject] = []
    h_max = 3.0
    for i in range(n_obstacles):
        # 与起点保持 >1.5 m
        while True:
            x = rng.uniform(-area, area)
            y = rng.uniform(-area, area)
            if np.hypot(x, y) > 1.5:
                break
        kind = ["box", "cylinder", "sphere"][i % 3]
        h = rng.uniform(0.6, h_max)
        if kind == "box":
            sx = rng.uniform(0.15, 0.5)
            sy = rng.uniform(0.15, 0.5)
            objs.append(SceneObject(f"obstacle_box{i}", "box",
                                    (x, y, h / 2), (sx, sy, h / 2),
                                    (0.55, 0.32, 0.25, 1.0)))
        elif kind == "cylinder":
            r = rng.uniform(0.12, 0.35)
            objs.append(SceneObject(f"obstacle_cyl{i}", "cylinder",
                                    (x, y, h / 2), (r, h / 2),
                                    (0.35, 0.45, 0.35, 1.0)))
        else:
            r = rng.uniform(0.15, 0.4)
            objs.append(SceneObject(f"obstacle_sph{i}", "sphere",
                                    (x, y, r), (r,),
                                    (0.45, 0.35, 0.5, 1.0)))
    return Scene(name="obstacle_field", objects=objs,
                 notes=f"随机障碍场：{n_obstacles} 个障碍，"
                       f"种子 {seed}，范围 ±{area} m，高度 ≤{h_max} m。")


def corridor(seed: int = 0, length: float = 24.0, width: float = 3.0) -> Scene:
    """室内走廊：两侧连续墙面，用于 VIO / SLAM 的弱纹理与重复结构场景。"""
    objs: List[SceneObject] = []
    h = 3.0
    t = 0.25
    half = width / 2
    seg = 4.0
    n = int(length // seg)
    for i in range(n):
        x = -length / 2 + seg * (i + 0.5)
        for side, sy in ((0, +1), (1, -1)):
            objs.append(SceneObject(
                f"wall_corridor_{side}_{i}", "box",
                (x, sy * (half + t / 2), h / 2), (seg / 2, t / 2, h / 2),
                (0.78, 0.76, 0.72, 1.0)))
    # 尽头封口
    objs.append(SceneObject("wall_end", "box",
                            (length / 2 + t / 2, 0.0, h / 2), (t / 2, half, h / 2),
                            (0.78, 0.76, 0.72, 1.0)))
    # 立柱：为 SLAM 提供可辨识特征
    for i in range(3):
        x = -length / 2 + 6.0 + i * 7.0
        objs.append(SceneObject(f"obstacle_pillar{i}", "cylinder",
                                (x, 0.6, h / 2), (0.12, h / 2),
                                (0.35, 0.33, 0.30, 1.0)))
    return Scene(name="corridor", ground_size=(length + 6, width + 6),
                 objects=objs,
                 drone_pos=(-length / 2 + 1.5, 0.0, 1.2),
                 notes="室内走廊：长直墙面 + 立柱，用于视觉里程计与 SLAM 评估。")


def gate_track(seed: int = 0, n_gates: int = 8, spacing: float = 5.0,
               jitter: float = 1.2) -> Scene:
    """穿越门赛道：沿 +x 方向排布一连串矩形门，用于端到端竞速。"""
    rng = np.random.default_rng(seed)
    objs: List[SceneObject] = []
    # 门框内净空 2.2 m × 2.2 m，边框截面 0.25 m
    opening = 2.2
    bar = 0.25
    half = opening / 2 + bar / 2
    for i in range(n_gates):
        x = spacing * (i + 1)
        y = float(rng.uniform(-jitter, jitter))
        z = float(2.0 + rng.uniform(-jitter, jitter))
        # 左/右立柱
        for s, sy in ((0, +1), (1, -1)):
            objs.append(SceneObject(f"gate_post_{i}_{s}", "box",
                                    (x, y + sy * half, z), (bar / 2, bar / 2, half),
                                    (0.85, 0.35, 0.15, 1.0)))
        # 上/下横梁
        for s, sz in ((0, +1), (1, -1)):
            objs.append(SceneObject(f"gate_bar_{i}_{s}", "box",
                                    (x, y, z + sz * half), (bar / 2, half, bar / 2),
                                    (0.85, 0.35, 0.15, 1.0)))
    return Scene(name="gate_track", ground_size=(n_gates * spacing + 20, 40.0),
                 objects=objs,
                 drone_pos=(-2.0, 0.0, 2.0),
                 notes=f"穿越门赛道：{n_gates} 个门，间距 {spacing} m，"
                       f"门内净空 {opening} m，位置抖动 ±{jitter} m。")


def urban_block(seed: int = 0, n_buildings: int = 14,
                n_trees: int = 10, n_cars: int = 6) -> Scene:
    """城市街区：建筑 + 植被 + 车辆，语义类别最丰富（语义建图与导航）。"""
    rng = np.random.default_rng(seed)
    objs: List[SceneObject] = []
    # 建筑沿道路两侧成排布置
    for i in range(n_buildings):
        side = 1 if i % 2 == 0 else -1
        x = -26.0 + 7.0 * (i // 2)
        y = side * rng.uniform(7.0, 13.0)
        w = rng.uniform(2.0, 3.5)
        d = rng.uniform(2.0, 3.5)
        h = rng.uniform(4.0, 11.0)
        objs.append(SceneObject(f"building_{i}", "box",
                                (x, y, h / 2), (w / 2, d / 2, h / 2),
                                tuple(rng.uniform(0.55, 0.85, 3)) + (1.0,)))
    for i in range(n_trees):
        x = float(rng.uniform(-24.0, 14.0))
        y = float(rng.uniform(-5.5, 5.5))
        h = float(rng.uniform(2.5, 5.0))
        objs.append(SceneObject(f"tree_trunk{i}", "cylinder",
                                (x, y, h * 0.3), (0.13, h * 0.3),
                                (0.30, 0.22, 0.14, 1.0)))
        objs.append(SceneObject(f"tree_crown{i}", "sphere",
                                (x, y, h * 0.72), (h * 0.30,),
                                (0.18, 0.42, 0.20, 1.0)))
    for i in range(n_cars):
        x = float(rng.uniform(-24.0, 14.0))
        y = float(rng.uniform(-2.5, 2.5))
        objs.append(SceneObject(f"car_body{i}", "box",
                                (x, y, 0.45), (0.9, 0.42, 0.35),
                                tuple(rng.uniform(0.2, 0.9, 3)) + (1.0,)))
    return Scene(name="urban_block", ground_size=(70.0, 70.0), objects=objs,
                 drone_pos=(-28.0, 0.0, 3.0),
                 notes=f"城市街区：{n_buildings} 栋建筑、{n_trees} 棵树、"
                       f"{n_cars} 辆车。语义类别最丰富，含建筑/植被/车辆。")


SCENE_BUILDERS = {
    "open_ground": open_ground,
    "obstacle_field": obstacle_field,
    "corridor": corridor,
    "gate_track": gate_track,
    "urban_block": urban_block,
}


def make_scene(name: str, **kwargs) -> Scene:
    """按名字构造场景，多余的参数透传给对应的构造器。"""
    if name not in SCENE_BUILDERS:
        raise KeyError(f"未知场景 {name!r}，可选：{sorted(SCENE_BUILDERS)}")
    return SCENE_BUILDERS[name](**kwargs)


# --------------------------------------------------------------------------- #
# 组装成 MuJoCo 模型
# --------------------------------------------------------------------------- #
_GROUND_RGBA = "0.34 0.40 0.32 1.0"


def scene_to_xml(scene: Scene) -> str:
    """把四旋翼模板 + 场景对象合并成一份完整 MJCF 字符串。"""
    tree = ET.parse(QUADROTOR_XML)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("四旋翼模板缺少 <worldbody>")

    # 1) 地面：模板不含地面，由场景注入
    worldbody.insert(0, ET.fromstring(
        f'<geom name="floor_main" type="plane" '
        f'size="{scene.ground_size[0]:.1f} {scene.ground_size[1]:.1f} 0.1" '
        f'rgba="{_GROUND_RGBA}" material="grid" condim="3"/>'
    ))

    # 2) 场景对象
    for obj in scene.objects:
        worldbody.append(ET.fromstring(obj.to_xml()))

    # 3) 无人机初始位置
    base = worldbody.find("./body[@name='base_link']")
    if base is not None:
        x, y, z = scene.drone_pos
        base.set("pos", f"{x:.4f} {y:.4f} {z:.4f}")
        if abs(scene.drone_yaw) > 1e-9:
            half = scene.drone_yaw / 2.0
            worldbody.find("./body[@name='base_link']").set(
                "quat", f"{np.cos(half):.6f} 0 0 {np.sin(half):.6f}")

    # 4) 时间步覆盖
    if scene.timestep is not None:
        opt = root.find("option")
        if opt is not None:
            opt.set("timestep", f"{scene.timestep:g}")

    xml = ET.tostring(root, encoding="unicode")
    return '<?xml version="1.0" encoding="utf-8"?>\n' + xml
