#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""解析地形高程场。

地形用「缓坡 + 若干各向同性高斯隆起」描述：

    h(x, y) = base + tan(slope)·x + Σ Aᵢ·exp(−((x−xᵢ)² + (y−yᵢ)²) / (2σᵢ²))

选这种形式而不是查表高程图，是因为它**处处可微且导数有闭式**——本模块要判断
「以速度 v 沿 +x 飞过这道山脊，需要多大爬升率」，需要的是坡度而不是采样点。
每一段地形都可以手工验算，也方便第三人复现。

坐标系与仓库一致：ENU，x 东、y 北、z 上，单位米、弧度。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, Tuple

import numpy as np

#: 采样坡度时的默认步长（米）
SLOPE_STEP_M = 0.5


def _as_pair(x, y):
    """标量/数组混用时的统一出口：两个输入都是标量就返回标量。"""
    xs = np.asarray(x, dtype=float)
    ys = np.asarray(y, dtype=float)
    scalar = xs.ndim == 0 and ys.ndim == 0
    return xs, ys, scalar


@dataclass(frozen=True)
class Ridge:
    """各向同性高斯隆起（幅值为负即沟谷）。"""

    x: float
    y: float
    amplitude: float
    sigma: float

    def __post_init__(self) -> None:
        for key, val in (("x", self.x), ("y", self.y),
                         ("amplitude", self.amplitude), ("sigma", self.sigma)):
            if not math.isfinite(float(val)):
                raise ValueError(f"山脊参数 {key} 必须是有限数，收到 {val!r}")
        if self.sigma <= 0.0:
            raise ValueError(f"山脊半宽 sigma 必须为正，收到 {self.sigma!r}")
        if self.amplitude == 0.0:
            raise ValueError("山脊幅值为 0 没有意义，请直接删掉这个 Ridge")

    # ---------------------------------------------------------------- 形状
    def bump(self, x, y):
        """相对中心的高度贡献。"""
        xs, ys, _ = _as_pair(x, y)
        dx = xs - self.x
        dy = ys - self.y
        return self.amplitude * np.exp(-(dx * dx + dy * dy) / (2.0 * self.sigma ** 2))

    def dsdx(self, x, y):
        """∂h/∂x = bump · (−Δx/σ²)。"""
        xs, ys, _ = _as_pair(x, y)
        dx = xs - self.x
        dy = ys - self.y
        return self.bump(xs, ys) * (-dx / self.sigma ** 2)

    def dsdy(self, x, y):
        """∂h/∂y = bump · (−Δy/σ²)。"""
        xs, ys, _ = _as_pair(x, y)
        dx = xs - self.x
        dy = ys - self.y
        return self.bump(xs, ys) * (-dy / self.sigma ** 2)

    def peak_slope(self) -> float:
        """沿 x 的最大坡度 |∂h/∂x|，出现在 x = x₀ − σ 处，等于 |A|/(σ√e)。"""
        return abs(self.amplitude) / (self.sigma * math.sqrt(math.e))


@dataclass(frozen=True)
class TerrainField:
    """地形高程场。``slope_deg`` 是沿 +x 的恒定上坡角。"""

    name: str
    base_m: float = 0.0
    slope_deg: float = 0.0
    ridges: Tuple[Ridge, ...] = ()

    def __post_init__(self) -> None:
        if not math.isfinite(self.base_m):
            raise ValueError(f"基准高度必须是有限数，收到 {self.base_m!r}")
        if not -60.0 < self.slope_deg < 60.0:
            raise ValueError(f"坡度超出 ±60°，收到 {self.slope_deg!r}")
        object.__setattr__(self, "ridges", tuple(self.ridges))

    # ---------------------------------------------------------------- 几何
    @property
    def slope(self) -> float:
        """恒定上坡的 tan 值（无量纲）。"""
        return math.tan(math.radians(self.slope_deg))

    def elevation(self, x, y=0.0):
        """地形高程 h(x, y)，标量进标量出、数组进数组出。"""
        xs, ys, scalar = _as_pair(x, y)
        # 先按广播后的形状铺底：没有山脊时也要保证 (ny, nx) 的二维输出
        shape = np.broadcast(xs, ys).shape
        h = np.full(shape, float(self.base_m)) + self.slope * xs
        for ridge in self.ridges:
            h = h + ridge.bump(xs, ys)
        return float(h) if scalar else h

    def gradient(self, x, y=0.0):
        """(∂h/∂x, ∂h/∂y)，用于把水平速度换成地形变化率。"""
        xs, ys, scalar = _as_pair(x, y)
        shape = np.broadcast(xs, ys).shape
        dhdx = np.full(shape, self.slope, dtype=float)
        dhdy = np.zeros(shape, dtype=float)
        for ridge in self.ridges:
            dhdx = dhdx + ridge.dsdx(xs, ys)
            dhdy = dhdy + ridge.dsdy(xs, ys)
        if scalar:
            return float(dhdx), float(dhdy)
        return dhdx, dhdy

    def terrain_rate(self, x, vx, vy=0.0, y=0.0):
        """地形沿航迹的变化率 ḣ = ∇h·v（米/秒），等价于跟随所需的爬升率。"""
        dhdx, dhdy = self.gradient(x, y)
        return dhdx * vx + dhdy * vy

    def profile(self, x0: float, x1: float, n: int = 200, y: float = 0.0):
        """沿 +x 的剖面，返回 (xs, hs)。"""
        if n < 2:
            raise ValueError(f"剖面点数至少 2 个，收到 {n}")
        xs = np.linspace(float(x0), float(x1), int(n))
        return xs, self.elevation(xs, y)

    def max_slope(self, x0: float, x1: float, y: float = 0.0,
                  step: float = SLOPE_STEP_M) -> float:
        """[x0, x1] 上沿 +x 的最大坡度（取绝对值）。"""
        if x1 <= x0:
            raise ValueError(f"要求 x1 > x0，收到 {x0} → {x1}")
        n = max(2, int(math.ceil((x1 - x0) / max(step, 1e-6))) + 1)
        xs = np.linspace(float(x0), float(x1), n)
        dhdx, _ = self.gradient(xs, y)
        return float(np.max(np.abs(dhdx)))

    def required_climb_rate(self, speed_mps: float, x0: float, x1: float,
                            y: float = 0.0, step: float = SLOPE_STEP_M) -> float:
        """以 ``speed_mps`` 沿 +x 精确贴合地形所需的最大爬升率（米/秒）。"""
        if speed_mps < 0:
            raise ValueError(f"速度必须非负，收到 {speed_mps!r}")
        return abs(float(speed_mps)) * self.max_slope(x0, x1, y, step)

    def max_speed_for_climb(self, climb_rate_mps: float, x0: float, x1: float,
                            y: float = 0.0, step: float = SLOPE_STEP_M) -> float:
        """爬升率上限给定时，能精确贴合该段地形的最大速度（米/秒）。

        这是本模块给设计者的约束：``v ≤ ṙ_max / max|∂h/∂x|``。恒坡上没有任何
        控制律能绕开它——想飞更快只能抬 AGL 设定或降低坡度。
        """
        if climb_rate_mps <= 0:
            raise ValueError(f"爬升率上限必须为正，收到 {climb_rate_mps!r}")
        m = self.max_slope(x0, x1, y, step)
        return float("inf") if m <= 1e-12 else float(climb_rate_mps) / m

    def clearance(self, x, z, y=0.0):
        """离地间隙 z − h(x, y)，负值表示已经侵入地形。"""
        return np.asarray(z, dtype=float) - np.asarray(self.elevation(x, y), dtype=float)

    def summary(self) -> Dict[str, float]:
        """给 JSON 用的地形摘要（不含区间相关量，便于跨算例比较）。"""
        xs = np.linspace(0.0, 600.0, 1201)
        hs = self.elevation(xs, 0.0)
        dhdx, dhdy = self.gradient(xs, 0.0)
        return {
            "name": self.name,
            "base_m": float(self.base_m),
            "slope_deg": float(self.slope_deg),
            "ridges": len(self.ridges),
            "min_elevation_m": float(np.min(hs)),
            "max_elevation_m": float(np.max(hs)),
            "max_slope_deg": float(math.degrees(math.atan(np.max(np.abs(dhdx))))),
            "max_slope_y_deg": float(math.degrees(math.atan(np.max(np.abs(dhdy))))),
        }


# --------------------------------------------------------------------------- #
# 预设地形
# --------------------------------------------------------------------------- #
def flat() -> TerrainField:
    """全平地面，作为对照。"""
    return TerrainField(name="flat")


def gentle_slope(deg: float = 6.0) -> TerrainField:
    """恒定缓坡：用来验「跟随误差 = v·tanθ·(1/Kp + τ)」这条解析式。"""
    return TerrainField(name=f"slope_{deg:g}deg", slope_deg=float(deg))


def ridge_crossing() -> TerrainField:
    """一道主山脊 + 一道缓脊。

    主脊 30 m / 半宽 25 m：坡度峰值 0.73（36°），8 m/s 时需要 5.8 m/s 的瞬时
    爬升率，远超 3 m/s 预算——纯反馈在这里必然挖坑；前馈靠「提前 80 m 起爬」
    把峰值需求摊成平均需求，从而过得去。
    """
    return TerrainField(
        name="ridge_crossing",
        ridges=(Ridge(280.0, 0.0, 30.0, 25.0),
                Ridge(480.0, 0.0, 15.0, 60.0)),
    )


def mixed() -> TerrainField:
    """混合地形：4° 缓坡 + 两道山脊 + 一道沟谷（低空巡检常见组合）。"""
    return TerrainField(
        name="mixed",
        slope_deg=4.0,
        ridges=(Ridge(240.0, 0.0, 26.0, 40.0),
                Ridge(430.0, 0.0, 36.0, 32.0),
                Ridge(545.0, 0.0, -14.0, 32.0)),
    )


def steep_slope(deg: float = 14.0) -> TerrainField:
    """陡坡：用来看爬升率上限被顶到以后三种策略一起失效。"""
    return TerrainField(name=f"slope_{deg:g}deg", slope_deg=float(deg))


PRESETS = {
    "flat": flat,
    "gentle_slope": gentle_slope,
    "ridge_crossing": ridge_crossing,
    "mixed": mixed,
    "steep_slope": steep_slope,
}


def get_terrain(name: str) -> TerrainField:
    """按名字取预设地形。"""
    try:
        factory = PRESETS[name]
    except KeyError:
        raise KeyError(
            f"未知地形 {name!r}，可选：{', '.join(sorted(PRESETS))}"
        ) from None
    return factory()


def list_terrains() -> Iterable[str]:
    return tuple(sorted(PRESETS))
