#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SLAM 基准世界：均匀分布的细立柱 + 少量遮挡物，以及一条八字形航线。

立柱的**真值位置**由本模块生成并保留，评测时用来算路标位置误差；
算法侧只能经激光雷达看到它们。底座自带的 ``obstacle_field`` 圆柱半径、
高度、分布随机，最低 0.66 m，1.5 m 高度处观测时有时无，路标关联不稳定，
因此这里用固定布局。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

import numpy as np

from uav_mujoco_sim.core.scenes import Scene, SceneObject

__all__ = ["Landmark", "World", "build_world", "LoopTrajectory",
           "POLE_RGBA", "OBSTACLE_RGBA"]


POLE_RGBA = (0.85, 0.35, 0.20, 1.0)
OBSTACLE_RGBA = (0.45, 0.48, 0.52, 1.0)


@dataclass
class Landmark:
    """一个可被观测到的立柱（真值）。"""

    idx: int
    pos: np.ndarray            # (2,) 水平位置，世界系 ENU
    radius: float
    height: float


@dataclass
class World:
    scene: Scene
    landmarks: List[Landmark]
    name: str = "slam_world"

    @property
    def positions(self) -> np.ndarray:
        return np.asarray([l.pos for l in self.landmarks], dtype=float)

    @property
    def radii(self) -> np.ndarray:
        return np.asarray([l.radius for l in self.landmarks], dtype=float)

    def describe(self) -> Dict[str, object]:
        p = self.positions
        return {"n_landmarks": len(self.landmarks),
                "extent_x": [float(p[:, 0].min()), float(p[:, 0].max())],
                "extent_y": [float(p[:, 1].min()), float(p[:, 1].max())],
                "radius": [float(self.radii.min()), float(self.radii.max())]}


def build_world(seed: int = 0, n_poles: int = 30, area: float = 13.0,
                n_pillars: int = 6) -> World:
    """生成基准世界。

    ``n_poles`` 根红色立柱（路标）按抖动网格铺开，保证任意位置都能看到
    3 根以上、且不会两两靠得太近；``n_pillars`` 个灰色大方柱只做遮挡，
    不参与路标观测。
    """
    rng = np.random.default_rng(seed)
    objs: List[SceneObject] = []
    marks: List[Landmark] = []

    # 抖动网格：先按行列放，再加 ±35% 格距的抖动，避免出现"完美直线"这种
    # 让最小二乘数据关联退化的构型
    side = int(math.ceil(math.sqrt(n_poles)))
    step = 2.0 * area / max(side, 1)
    k = 0
    for i in range(side):
        for j in range(side):
            if k >= n_poles:
                break
            x = -area + step * (i + 0.5) + rng.uniform(-0.35, 0.35) * step
            y = -area + step * (j + 0.5) + rng.uniform(-0.35, 0.35) * step
            if math.hypot(x, y) < 2.2:          # 别堵住航线中心
                continue
            r = float(rng.uniform(0.09, 0.16))
            h = float(rng.uniform(2.4, 3.0))
            name = "pole_%02d" % k
            objs.append(SceneObject(name=name, kind="cylinder",
                                    pos=(x, y, h / 2), size=(r, h / 2),
                                    rgba=POLE_RGBA))
            marks.append(Landmark(k, np.array([x, y]), r, h))
            k += 1

    for i in range(n_pillars):
        ang = 2 * math.pi * i / n_pillars + 0.3
        d = area * 1.32
        x, y = d * math.cos(ang), d * math.sin(ang)
        w = float(rng.uniform(0.5, 0.9))
        h = float(rng.uniform(2.6, 3.6))
        objs.append(SceneObject(name="pillar_%d" % i, kind="box",
                                pos=(x, y, h / 2), size=(w, w, h / 2),
                                rgba=OBSTACLE_RGBA))

    scene = Scene(name="slam_world", ground_size=(70.0, 70.0), objects=objs,
                  drone_pos=(0.0, 0.0, 1.6), drone_yaw=0.0,
                  notes="SLAM 基准：30 根立柱（路标）+ 6 个遮挡方柱")
    return World(scene=scene, landmarks=marks)


# --------------------------------------------------------------------------- #
class LoopTrajectory:
    """八字形闭环航线，解析的位置/速度/加速度/偏航。

    八字的自交点天然提供一次**回环**：前端漂移积累到那里时，三个后端的
    表现差异最明显。振幅 9 × 6.5 m，周期 40 s，巡航速度约 1.4 m/s。
    """

    def __init__(self, ax: float = 9.0, ay: float = 6.5, height: float = 1.7,
                 period: float = 40.0, dz: float = 0.25):
        self.ax, self.ay = float(ax), float(ay)
        self.z0, self.dz = float(height), float(dz)
        self.period = float(period)
        self.w = 2.0 * math.pi / self.period

    @property
    def duration(self) -> float:
        return self.period

    def eval(self, t: float):
        w, ax, ay = self.w, self.ax, self.ay
        s1, c1 = math.sin(w * t), math.cos(w * t)
        s2, c2 = math.sin(2 * w * t), math.cos(2 * w * t)
        pos = np.array([ax * s1, ay * s2, self.z0 + self.dz * math.sin(0.5 * w * t)])
        vel = np.array([ax * w * c1, 2 * ay * w * c2,
                        0.5 * self.dz * w * math.cos(0.5 * w * t)])
        acc = np.array([-ax * w * w * s1, -4 * ay * w * w * s2,
                        -0.25 * self.dz * w * w * math.sin(0.5 * w * t)])
        yaw = math.atan2(vel[1], vel[0])
        sp2 = float(vel[0] ** 2 + vel[1] ** 2) + 1e-12
        yaw_rate = (vel[0] * acc[1] - vel[1] * acc[0]) / sp2
        return pos, vel, acc, yaw, yaw_rate

    def sample(self, n: int, t0: float = 0.0) -> np.ndarray:
        return np.asarray([self.eval(t0 + self.period * k / n)[0] for k in range(n)])


def circles_trajectory(radius: float = 8.0, height: float = 1.7,
                       period: float = 30.0) -> LoopTrajectory:
    """纯圆航线：没有回环，用来对比"回环对精度的影响"。"""
    tj = LoopTrajectory(ax=radius, ay=0.0, height=height, period=period)
    tj._circle = True
    return tj


def landmark_dict(landmarks: Sequence[Landmark]) -> Dict[int, np.ndarray]:
    return {l.idx: l.pos.copy() for l in landmarks}
