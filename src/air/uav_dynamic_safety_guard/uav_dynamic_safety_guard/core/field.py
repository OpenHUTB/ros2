#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""动态障碍场：静态障碍 + 圆周运动的障碍，以及按时取几何体的接口。

运动障碍的参数（圆心、半径、角速度、相位）对教师层可见，因此教师的外推
是精确的；学生只能看到三帧距离栅格，必须自己把运动从历史里读出来。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .geometry import (DRONE_RADIUS, Batch, Primitive, clearance_at,
                       clearance_points, clearance_points_batch, merge_batch,
                       stack_batch)

__all__ = ["Mover", "DynamicField", "safety_field", "sample_endpoints",
           "FIELD_SIZES"]


@dataclass
class Mover:
    """沿圆周匀速运动的柱状障碍：``pos(t) = center + R (cos φ, sin φ)``。"""

    name: str
    center: Tuple[float, float]
    radius: float
    omega: float                      # rad/s，正负表示绕行方向
    phase: float
    size: Tuple[float, float]         # (半径, 半高)
    kind: str = "cylinder"

    def pos_at(self, t: float) -> np.ndarray:
        a = self.phase + self.omega * float(t)
        return np.array([self.center[0] + self.radius * math.cos(a),
                         self.center[1] + self.radius * math.sin(a),
                         self.size[1]], float)

    def prim_at(self, t: float) -> Primitive:
        return (self.kind, self.pos_at(t), np.asarray(self.size, float))

    def speed(self) -> float:
        return abs(float(self.omega)) * float(self.radius)


@dataclass
class DynamicField:
    """一个场景：静态几何体 + 运动障碍 + 地面（z = 0）。"""

    name: str
    static: List[Primitive] = field(default_factory=list)
    movers: List[Mover] = field(default_factory=list)
    area: float = 9.0
    seed: int = 0

    def __post_init__(self) -> None:
        self._static_batch: Batch = stack_batch(self.static)
        self._cache_t: Optional[float] = None
        self._cache_batch: Optional[Batch] = None

    # ------------------------------------------------------------------ 几何
    def primitives_at(self, t: float = 0.0) -> List[Primitive]:
        if not self.movers:
            return list(self.static)
        return list(self.static) + [m.prim_at(t) for m in self.movers]

    def batch_at(self, t: float = 0.0) -> Batch:
        """``t`` 时刻的障碍批（静态部分预堆叠，运动障碍按题设轨迹求值）。"""
        if not self.movers:
            return self._static_batch
        t = float(t)
        if self._cache_t is not None and abs(t - self._cache_t) < 1e-9:
            assert self._cache_batch is not None
            return self._cache_batch
        mb = stack_batch([m.prim_at(t) for m in self.movers])
        self._cache_batch = merge_batch(self._static_batch, mb)
        self._cache_t = t
        return self._cache_batch

    def clearance_at(self, p: Sequence[float], t: float = 0.0) -> float:
        pts = np.asarray(p, float).reshape(1, 3)
        return float(clearance_points_batch(self.batch_at(t), pts)[0])

    def clearance_points(self, pts: np.ndarray, t: float = 0.0) -> np.ndarray:
        return clearance_points_batch(self.batch_at(t), pts)

    def mover_positions(self, t: float) -> np.ndarray:
        if not self.movers:
            return np.zeros((0, 3))
        return np.asarray([m.pos_at(t) for m in self.movers], float)

    def speeds(self) -> np.ndarray:
        return np.asarray([m.speed() for m in self.movers], float)

    def n_moving(self) -> int:
        return len(self.movers)

    # ------------------------------------------------------------------ 约束
    def in_bounds(self, p: Sequence[float], margin: float = 0.0) -> bool:
        p = np.asarray(p, float)
        lim = float(self.area) + float(margin)
        return bool(abs(p[0]) <= lim and abs(p[1]) <= lim)

    def label(self) -> str:
        return "%s/static%d/moving%d" % (self.name, len(self.static),
                                         len(self.movers))


FIELD_SIZES = {"safety_field": "动态障碍场"}


def safety_field(seed: int = 0, n_static: int = 18, n_moving: int = 5,
                 area: float = 9.0, mover_speed: Tuple[float, float] = (0.4, 1.0),
                 mover_path: Tuple[float, float] = (1.4, 3.2),
                 static_height: Tuple[float, float] = (1.2, 2.8),
                 mover_height: Tuple[float, float] = (1.2, 2.0)) -> DynamicField:
    """动态障碍场：``n_static`` 个静止盒/柱 + ``n_moving`` 个绕圈柱体。

    静止障碍高度下限 1.2 m，保证 ±15° 三层激光都能打到；运动障碍同样高
    1.2 m 以上，避免出现“雷达层间盲区”这种与本题无关的失败模式。
    """
    rng = np.random.default_rng(int(seed) + 77)
    static: List[Primitive] = []
    for i in range(int(n_static)):
        while True:
            x = float(rng.uniform(-area, area))
            y = float(rng.uniform(-area, area))
            if math.hypot(x, y) > 1.6 and max(abs(x), abs(y)) < area - 0.6:
                break
        h = float(rng.uniform(*static_height))
        if i % 2 == 0:
            sx = float(rng.uniform(0.18, 0.45))
            sy = float(rng.uniform(0.18, 0.45))
            static.append(("box", np.array([x, y, h / 2]),
                           np.array([sx, sy, h / 2])))
        else:
            r = float(rng.uniform(0.15, 0.35))
            static.append(("cylinder", np.array([x, y, h / 2]),
                           np.array([r, h / 2])))
    movers: List[Mover] = []
    lim = area - 3.4
    for i in range(int(n_moving)):
        cx = cy = 0.0
        path_r = 2.0
        for _ in range(200):
            cx = float(rng.uniform(-lim, lim))
            cy = float(rng.uniform(-lim, lim))
            path_r = float(rng.uniform(*mover_path))
            if math.hypot(cx, cy) > 2.6:
                break
        v = float(rng.uniform(*mover_speed))
        omega = v / max(path_r, 1e-6) * (1.0 if rng.random() < 0.5 else -1.0)
        h = float(rng.uniform(*mover_height))
        r = float(rng.uniform(0.18, 0.32))
        movers.append(Mover("mover_cyl%d" % i, (cx, cy), path_r, omega,
                            float(rng.uniform(0.0, 2.0 * np.pi)),
                            (r, h / 2)))
    return DynamicField(name="safety_field", static=static, movers=movers,
                        area=float(area), seed=int(seed))


def sample_endpoints(fld: DynamicField, rng: np.random.Generator,
                     radius: float = 7.5, z: float = 1.6,
                     tries: int = 300) -> Tuple[np.ndarray, np.ndarray]:
    """对穿障碍场的起点/终点：初始间距 >0.9 m，直线扫掠 >1.0 m。"""
    base = fld.batch_at(0.0)
    for _ in range(int(tries)):
        th = float(rng.uniform(0.0, 2.0 * math.pi))
        r1 = float(rng.uniform(radius - 1.2, radius))
        r2 = float(rng.uniform(radius - 1.2, radius))
        start = np.array([r1 * math.cos(th), r1 * math.sin(th),
                          z + float(rng.uniform(-0.25, 0.25))])
        goal = np.array([-r2 * math.cos(th), -r2 * math.sin(th),
                         z + float(rng.uniform(-0.25, 0.25))])
        d = goal - start
        ts = np.linspace(0.0, 1.0, 9)[:, None]
        pts = start[None, :] + d[None, :] * ts
        if (clearance_points_batch(base, start[None, :])[0] > 0.9
                and clearance_points_batch(base, goal[None, :])[0] > 0.6
                and float(clearance_points_batch(base, pts).min()) > 1.0):
            return start, goal
    return (np.array([-radius, 0.0, z]), np.array([radius, 0.0, z]))


def field_clearance_min(fld: DynamicField, p: Sequence[float],
                        t: float = 0.0) -> float:
    return float(clearance_at(fld.primitives_at(t), p) + DRONE_RADIUS)
