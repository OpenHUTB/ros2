#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""算例定义：把「地形 + 速度 + AGL 设定 + 传感器/DEM 误差」打成一个不可变对象。"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Tuple

from .altimeter import AltimeterConfig
from .control import VerticalLimits
from .dem import DEGRADED_DEM, IDEAL_DEM, PRACTICAL_DEM, DemConfig
from .terrain import TerrainField, get_terrain


@dataclass(frozen=True)
class Case:
    name: str
    title: str
    terrain_name: str
    speed_mps: float
    agl_set_m: float = 15.0
    safety_m: float = 5.0
    length_m: float = 600.0
    ramp_s: float = 4.0
    seed: int = 20261111
    dem: DemConfig = PRACTICAL_DEM
    limits: VerticalLimits = VerticalLimits()
    altimeter: AltimeterConfig = AltimeterConfig()

    def __post_init__(self) -> None:
        if not math.isfinite(self.speed_mps) or self.speed_mps <= 0:
            raise ValueError(f"巡航速度必须为正，收到 {self.speed_mps!r}")
        if not 1.0 <= self.agl_set_m <= 120.0:
            raise ValueError(f"AGL 设定应在 1~120 m，收到 {self.agl_set_m!r}")
        if not 0.0 <= self.safety_m < self.agl_set_m:
            raise ValueError(
                f"安全间隙应在 [0, AGL) 内，收到 {self.safety_m!r} / {self.agl_set_m!r}")
        if self.length_m <= 10.0:
            raise ValueError(f"航程太短，收到 {self.length_m!r}")
        if self.ramp_s <= 0.0:
            raise ValueError(f"加速段时间必须为正，收到 {self.ramp_s!r}")

    @property
    def terrain(self) -> TerrainField:
        return get_terrain(self.terrain_name)

    @property
    def warmup_s(self) -> float:
        """统计窗口起点：加速段 + 2 s 过渡。"""
        return self.ramp_s + 2.0

    @property
    def duration_s(self) -> float:
        return self.ramp_s + self.length_m / self.speed_mps + 2.0

    def required_climb_rate(self) -> float:
        """精确贴合该段地形所需的最大爬升率（米/秒）。"""
        return self.terrain.required_climb_rate(self.speed_mps, 0.0, self.length_m)

    def max_speed_for_climb(self) -> float:
        """受爬升率上限约束的最大速度（米/秒）。"""
        return self.terrain.max_speed_for_climb(self.limits.climb_rate_mps,
                                                0.0, self.length_m)

    def as_dict(self) -> Dict[str, object]:
        return {
            "name": self.name,
            "title": self.title,
            "terrain": self.terrain.summary(),
            "speed_mps": float(self.speed_mps),
            "agl_set_m": float(self.agl_set_m),
            "safety_m": float(self.safety_m),
            "length_m": float(self.length_m),
            "ramp_s": float(self.ramp_s),
            "seed": int(self.seed),
            "dem": self.dem.as_dict(),
            "altimeter": self.altimeter.as_dict(),
            "limits": {"climb_rate_mps": float(self.limits.climb_rate_mps),
                       "climb_accel_mps2": float(self.limits.climb_accel_mps2)},
            "required_climb_rate_mps": float(self.required_climb_rate()),
            "max_speed_for_climb_mps": float(self.max_speed_for_climb()),
        }


#: 主表用的四个算例。速度是按「爬升率上限 3 m/s」反推着选的：
#: 山脊段的峰值需求 30/45·√e⁻¹·v ≈ 0.40·v，8 m/s 时约 3.2 m/s（刚超预算），
#: 12 m/s 时约 4.9 m/s（明显超预算）——正好落在纯反馈失效的边界两侧。
CASES: Tuple[Case, ...] = (
    Case(name="ridge_8ms",
         title="单道山脊（30 m）· 8 m/s",
         terrain_name="ridge_crossing",
         speed_mps=8.0),
    Case(name="ridge_12ms",
         title="单道山脊（30 m）· 12 m/s",
         terrain_name="ridge_crossing",
         speed_mps=12.0),
    Case(name="mixed_10ms",
         title="混合地形（缓坡 + 双脊 + 沟谷）· 10 m/s",
         terrain_name="mixed",
         speed_mps=10.0),
    Case(name="slope6_10ms",
         title="恒定 6° 缓坡 · 10 m/s",
         terrain_name="gentle_slope",
         speed_mps=10.0),
)

#: DEM 质量三档（在同一个山脊算例上只跑 dem_preview），用来看「先验地形变差会掉多少间隙」。
DEM_LEVELS: Tuple[Tuple[str, DemConfig, str], ...] = (
    ("ideal", IDEAL_DEM, "理想 DEM（即真值）"),
    ("practical", PRACTICAL_DEM, "实用 DEM（2 m 网格 / 0.8 m 偏置 / 3 m 前移）"),
    ("degraded", DEGRADED_DEM, "劣化 DEM（8 m 网格 / 4 m 偏置 / 12 m 前移）"),
)

#: 速度扫描用的算例：固定地形与 AGL，只改速度。
SWEEP_CASE_NAME = "ridge_8ms"


def get_case(name: str) -> Case:
    for case in CASES:
        if case.name == name:
            return case
    raise KeyError(f"未知算例 {name!r}，可选：{', '.join(c.name for c in CASES)}")


def sweep_cases(speeds=(4.0, 6.0, 8.0, 10.0, 12.0, 14.0)) -> Tuple[Case, ...]:
    """同一地形上的速度扫描算例。"""
    base = get_case(SWEEP_CASE_NAME)
    out = []
    for v in speeds:
        out.append(Case(name=f"sweep_{v:g}ms",
                        title=f"山脊 · {v:g} m/s",
                        terrain_name=base.terrain_name,
                        speed_mps=float(v),
                        agl_set_m=base.agl_set_m,
                        safety_m=base.safety_m,
                        length_m=base.length_m,
                        ramp_s=base.ramp_s,
                        seed=base.seed,
                        dem=base.dem,
                        limits=base.limits,
                        altimeter=base.altimeter))
    return tuple(out)
