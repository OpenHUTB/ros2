#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视线几何：射线下视求交、掠射角、以及"每度姿态误差换多少米"的解析敏感度。

世界系 ENU（x 东 y 北 z 上），地面默认 z = 0。所有朝向都是 (3,) 单位向量，
约定 "视线" 由相机指向地面目标，因此 z 分量为负。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np

EPS = 1e-9
#: 视线几乎水平时的判定阈值（z 分量大于 -EPS 视为打不到地面）
MIN_DOWN_COS = 1e-3


def as_vec3(v: Sequence[float], name: str = "v") -> np.ndarray:
    """转成 (3,) float 数组，维度不对或含非有限值直接报错。"""
    a = np.asarray(v, dtype=float).reshape(-1)
    if a.shape != (3,):
        raise ValueError(f"{name} 必须是 3 维向量，收到 {a.shape}")
    if not np.all(np.isfinite(a)):
        raise ValueError(f"{name} 含非有限值：{a}")
    return a


def unit(v: Sequence[float], name: str = "v") -> np.ndarray:
    a = as_vec3(v, name)
    n = float(np.linalg.norm(a))
    if n < EPS:
        raise ValueError(f"{name} 是零向量，无法归一化")
    return a / n


@dataclass(frozen=True)
class Ray:
    """一条从相机出发的视线。"""

    origin: np.ndarray     # (3,) 相机位置（世界系）
    direction: np.ndarray  # (3,) 单位向量，指向地面

    def __post_init__(self) -> None:
        o = as_vec3(self.origin, "origin")
        d = unit(self.direction, "direction")
        object.__setattr__(self, "origin", o)
        object.__setattr__(self, "direction", d)

    # ------------------------------------------------------------------ 量
    @property
    def height(self) -> float:
        """相机相对地面的高度（m）。"""
        return float(self.origin[2])

    def ground_hit(self, ground_z: float = 0.0) -> np.ndarray:
        """射线与 z = ground_z 平面的交点。

        视线朝上、几乎水平或交点在相机后方时抛 ValueError——这几种情况
        在真实系统里都对应"看不到地面"，不能返回一个任意的数糊过去。
        """
        dz = float(self.direction[2])
        if dz > -MIN_DOWN_COS:
            raise ValueError(f"视线不朝下（dz={dz:.4f}），打不到地面")
        t = (float(ground_z) - float(self.origin[2])) / dz
        if t <= 0.0:
            raise ValueError(f"交点落在相机后方（t={t:.3f} s）")
        return self.origin + t * self.direction

    def ground_range(self, ground_z: float = 0.0) -> float:
        """交点与相机在地面上的水平距离（m）。"""
        p = self.ground_hit(ground_z)
        return float(np.hypot(p[0] - self.origin[0], p[1] - self.origin[1]))

    def slant_range(self, ground_z: float = 0.0) -> float:
        p = self.ground_hit(ground_z)
        return float(np.linalg.norm(p - self.origin))

    def grazing_angle_deg(self, ground_z: float = 0.0) -> float:
        """视线与竖直方向的夹角（0° 为正下方，越大越"掠射"）。"""
        h = float(self.origin[2]) - float(ground_z)
        if h <= 0.0:
            raise ValueError(f"相机不在地面上方（h={h:.3f} m）")
        return math.degrees(math.acos(min(1.0, max(-1.0, -float(self.direction[2])))))

    def sensitivity_m_per_deg(self, ground_z: float = 0.0) -> float:
        """视线角偏 1° 时地面交点的横向偏移（m）。

        地面射程 r = h·tanθ（θ 为视线与竖直方向的夹角），dr/dθ = h·sec²θ，
        即每弧度 h / cos²θ，而 cosθ 就是视线的竖直分量大小 |dz|。掠射角越大，
        同样的角误差被放大得越厉害——这是单站定位在高空小倾角时精度崩掉的根因。
        """
        h = float(self.origin[2]) - float(ground_z)
        if h <= 0.0:
            raise ValueError(f"相机不在地面上方（h={h:.3f} m）")
        c = abs(float(self.direction[2]))
        if c < MIN_DOWN_COS:
            raise ValueError("视线几乎水平，敏感度发散")
        return h / (c * c) * math.pi / 180.0

    def bearing_deg(self) -> float:
        """水平方位角（自 +x 逆时针，度）。"""
        return math.degrees(math.atan2(float(self.direction[1]),
                                       float(self.direction[0])))


def los_to(target: Sequence[float], origin: Sequence[float]) -> np.ndarray:
    """从 origin 指向 target 的单位视线。"""
    t = as_vec3(target, "target")
    o = as_vec3(origin, "origin")
    return unit(t - o, "los")


def make_ray(origin: Sequence[float], target: Sequence[float]) -> Ray:
    return Ray(origin=origin, direction=los_to(target, origin))


def rotate_by_angle_err(los: np.ndarray, err_deg: float,
                        axis: str = "y") -> np.ndarray:
    """把视线绕指定水平轴旋转一个小角度，用于蒙特卡洛打点。

    axis='y' 改变俯仰（前视方向），axis='x' 改变滚转（横向）。
    """
    a = math.radians(float(err_deg))
    c, s = math.cos(a), math.sin(a)
    if axis == "y":
        R = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    elif axis == "x":
        R = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    else:
        raise ValueError(f"axis 只支持 'x' / 'y'，收到 {axis!r}")
    return R @ unit(los)


def horizontal_intersection_angle_deg(rays: Sequence[Ray]) -> float:
    """两条（或多条中最小的一对）视线在水平面上的交会角（度，0~90）。

    交会角越小，两条近乎平行的射线就越难定出交点——这是双站交会的
    几何构型指标，和算法好坏无关。
    """
    if len(rays) < 2:
        raise ValueError("至少需要两条视线才能算交会角")
    dirs = []
    for r in rays:
        d = np.array([float(r.direction[0]), float(r.direction[1])])
        n = float(np.linalg.norm(d))
        if n < MIN_DOWN_COS:
            raise ValueError("视线几乎垂直向下，水平方向退化，交会角无意义")
        dirs.append(d / n)
    best = 90.0
    for i in range(len(dirs)):
        for j in range(i + 1, len(dirs)):
            c = abs(float(np.dot(dirs[i], dirs[j])))
            best = min(best, math.degrees(math.acos(min(1.0, c))))
    return best


def station_position(azimuth_deg: float, radius_m: float,
                     height_m: float) -> np.ndarray:
    """按极坐标给出观察点位置（方位角自 +x 逆时针）。"""
    a = math.radians(float(azimuth_deg))
    return np.array([radius_m * math.cos(a), radius_m * math.sin(a), height_m])


def pairwise_min_angle_deg(azimuths: Sequence[float]) -> float:
    """一组方位角里最小的水平夹角（度）。"""
    if len(azimuths) < 2:
        return float("nan")
    vals = []
    for i in range(len(azimuths)):
        for j in range(i + 1, len(azimuths)):
            d = abs((azimuths[i] - azimuths[j]) % 180.0)
            vals.append(min(d, 180.0 - d))
    return float(min(vals))


def circle_ellipse(cov: np.ndarray, n_sigma: float = 1.0) -> Tuple[float, float, float]:
    """由 2x2 协方差给出误差椭圆 (半长轴, 半短轴, 朝向角°)。"""
    C = np.asarray(cov, dtype=float)
    if C.shape != (2, 2):
        raise ValueError(f"协方差必须是 2x2，收到 {C.shape}")
    w, v = np.linalg.eigh(C)
    w = np.clip(w, 0.0, None)
    order = np.argsort(w)[::-1]
    w = w[order]
    v = v[:, order]
    a = n_sigma * math.sqrt(float(w[0]))
    b = n_sigma * math.sqrt(float(w[1]))
    ang = math.degrees(math.atan2(float(v[1, 0]), float(v[0, 0])))
    return a, b, ang
