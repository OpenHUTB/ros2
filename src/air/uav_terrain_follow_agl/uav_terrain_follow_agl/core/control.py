#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三种地形跟随策略。

三者都只输出「高度设定 + 爬升率设定」给底层几何控制器，不直接算推力——这样
换上任何飞控都不影响判断，也让「策略差异」不被姿态环掩盖。

=====================================  ==========================================
策略                                    用到的信息
=====================================  ==========================================
``constant_msl``（定高）                 起飞点高程，飞行中不再更新
``reactive_agl``（纯反馈）               只有（带延迟的）雷达高度表读数
``dem_preview``（前馈 + 慢环校正）        DEM 前视高程 + 雷达高度表积分校正
=====================================  ==========================================

关键差异不是「有没有反馈」，而是**信噪方向**：定高完全不用地形；纯反馈只能看见
脚下的地形，等它看见坡时已经开始掉高度；DEM 前馈在爬升率预算内**提前**把高度
做上去，雷达高度表则退化成慢速积分环，专门吃 DEM 的系统性偏置。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .altimeter import AltimeterConfig
from .dem import DigitalElevationModel, DemConfig
from .terrain import TerrainField


@dataclass(frozen=True)
class VerticalLimits:
    """垂直接近的物理约束：爬升率与加速度上限。"""

    climb_rate_mps: float = 3.0
    climb_accel_mps2: float = 2.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.climb_rate_mps) or self.climb_rate_mps <= 0:
            raise ValueError(f"爬升率上限必须为正，收到 {self.climb_rate_mps!r}")
        if not math.isfinite(self.climb_accel_mps2) or self.climb_accel_mps2 <= 0:
            raise ValueError(f"爬升加速度上限必须为正，收到 {self.climb_accel_mps2!r}")

    def limit(self, target: float, current: float, dt: float) -> float:
        """先按速率上限截断目标，再按加速度上限限制单步变化。"""
        target = float(np.clip(target, -self.climb_rate_mps, self.climb_rate_mps))
        dv = target - float(current)
        max_dv = self.climb_accel_mps2 * float(dt)
        return float(np.clip(current + float(np.clip(dv, -max_dv, max_dv)),
                             -self.climb_rate_mps, self.climb_rate_mps))


class TerrainFollowStrategy:
    """策略基类：只约定 reset / step 两个动作。"""

    name = "base"
    label = "base"

    def reset(self, terrain: TerrainField, dem: DigitalElevationModel,
              x0: float, z0: float, agl_set: float) -> None:
        raise NotImplementedError

    def step(self, t: float, dt: float, x: float, z: float, vx: float,
             agl_meas: Optional[float]) -> Tuple[float, float]:
        """返回 (高度设定 z_cmd, 爬升率设定 vz_cmd)。"""
        raise NotImplementedError


class ConstantMSL(TerrainFollowStrategy):
    """定高：锁死起飞高度，不知道地形在变。"""

    name = "constant_msl"
    label = "定高（MSL）"

    def reset(self, terrain, dem, x0, z0, agl_set):
        self._z = float(z0)

    def step(self, t, dt, x, z, vx, agl_meas):
        return self._z, 0.0


class ReactiveAGL(TerrainFollowStrategy):
    """纯反馈：PI 跟踪离地高度，只用延迟的雷达读数。"""

    name = "reactive_agl"
    label = "纯反馈（雷达）"

    def __init__(self, limits: VerticalLimits = VerticalLimits(),
                 kp: float = 1.2, ki: float = 0.25, i_limit: float = 8.0):
        if kp <= 0:
            raise ValueError(f"kp 必须为正，收到 {kp!r}")
        if ki < 0:
            raise ValueError(f"ki 必须非负，收到 {ki!r}")
        self.limits = limits
        self.kp = float(kp)
        self.ki = float(ki)
        self.i_limit = float(i_limit)

    def reset(self, terrain, dem, x0, z0, agl_set):
        self._z = float(z0)
        self._vz = 0.0
        self._i = 0.0
        self._agl_set = float(agl_set)
        self._error = 0.0

    def step(self, t, dt, x, z, vx, agl_meas):
        if agl_meas is None:                 # 还没出第一帧：维持当前高度
            return self._z, 0.0
        e = self._agl_set - float(agl_meas)
        self._error = e
        self._i = float(np.clip(self._i + self.ki * e * dt,
                                -self.i_limit, self.i_limit))
        vz = self.limits.limit(self.kp * e + self._i, self._vz, dt)
        self._vz = vz
        self._z += vz * dt
        return self._z, vz

    @property
    def error(self) -> float:
        return self._error


class DemPreview(TerrainFollowStrategy):
    """DEM 前馈 + 雷达高度表慢环校正。

    前馈不是「看前方固定几秒」，而是把爬升率预算显式用起来：

        z_ref(t) = max_{0 ≤ s ≤ L} [ DEM(x + s) + AGL_set − ṙ_max · s/v ] + trim

    意思是「现在的高度，至少要够我在剩余爬升预算里摸到前方 L 米内最高的那个
    地形点」。前方 30 m 的山脊、爬升率 3 m/s、速度 8 m/s 时，它会在离山脊
    还有 80 m（= 30/3 · 8）就开始爬，而不是等离地高度掉下来才反应——这正是
    纯反馈做不到的事。下降方向不设同等约束（掉高度不撞地），故只取单侧 max。

    ``trim`` 是雷达高度表的慢速积分：DEM 的垂直偏置是整片一致的，靠它吃掉；
    水平移位与网格误差则被「提前量 + 慢环」共同压住。
    """

    name = "dem_preview"
    label = "DEM 前馈 + 慢环"

    def __init__(self, limits: VerticalLimits = VerticalLimits(),
                 horizon_m: float = 150.0, step_m: float = 2.0,
                 kz: float = 1.5, ki: float = 0.02, trim_limit: float = 10.0,
                 profile_hook=None):
        if horizon_m <= 0:
            raise ValueError(f"前视距离必须为正，收到 {horizon_m!r}")
        if step_m <= 0 or step_m > horizon_m:
            raise ValueError(f"前视步长应在 (0, horizon] 内，收到 {step_m!r}")
        if kz <= 0:
            raise ValueError(f"kz 必须为正，收到 {kz!r}")
        if ki < 0:
            raise ValueError(f"ki 必须非负，收到 {ki!r}")
        self.limits = limits
        self.horizon_m = float(horizon_m)
        self.step_m = float(step_m)
        self.kz = float(kz)
        self.ki = float(ki)
        self.trim_limit = float(trim_limit)
        #: 在线节点用：直接给「从当前位置起、步长 step_m 的前视高程序列」，
        #: 这样节点不需要在机上保存整张 DEM，只要订阅一条剖面话题。
        self.profile_hook = profile_hook
        self._s = np.arange(0.0, self.horizon_m + 1e-9, self.step_m)

    def reset(self, terrain, dem, x0, z0, agl_set):
        if dem is None and self.profile_hook is None:
            raise ValueError("DemPreview 需要 DEM 或 profile_hook 二者之一")
        self._z = float(z0)
        self._vz = 0.0
        self._trim = 0.0
        self._agl_set = float(agl_set)
        self._dem = dem
        self._z_ref = float(z0)
        self._prev_ref = None
        self._dz_ref = 0.0

    def preview_reference(self, x: float, vx: float) -> float:
        """给定位置与地速，算可行的最低当前高度（不含 trim）。"""
        v = max(float(vx), 0.5)              # 起步阶段地速接近 0，用 0.5 m/s 兜底
        if self.profile_hook is not None:
            heights = np.asarray(self.profile_hook(), dtype=float)
            lead = self._s[:heights.size] / v
            return self.reference_from_profile(heights, lead)
        h = np.asarray(self._dem.height(float(x) + self._s, 0.0), dtype=float)
        return self.reference_from_profile(h, self._s / v)

    def reference_from_profile(self, heights, lead_s) -> float:
        """前视高程序列 + 各自提前量 → 当前应达到的最低高度。"""
        h = np.asarray(heights, dtype=float)
        lead = np.asarray(lead_s, dtype=float)
        if h.shape != lead.shape:
            raise ValueError(f"高度与前视时间形状不一致：{h.shape} vs {lead.shape}")
        if h.size == 0:
            raise ValueError("前视剖面为空")
        return float(np.max(h + self._agl_set - self.climb_capacity(lead)))

    def climb_capacity(self, lead_s) -> float:
        """给 ``lead_s`` 秒的提前量，靠爬升率上限能爬多高（含加速段损失）。

        加速段按 ``a = climb_accel`` 匀加速到 ``ṙ``，这段少爬 ``ṙ²/(2a)``——忽略
        它会让参考起步晚约 0.7 s，实测在 30 m 山脊上刚好差 2 m 间隙。
        """
        r = self.limits.climb_rate_mps
        a = self.limits.climb_accel_mps2
        t = np.maximum(np.asarray(lead_s, dtype=float), 0.0)
        return np.maximum(r * t - r * r / (2.0 * a), 0.0)

    def step(self, t, dt, x, z, vx, agl_meas):
        self._z_ref = self.preview_reference(x, vx) + self._trim
        # 参考本身在爬升，只给位置误差会让被控对象永远差 ṙ/Kz——必须把参考速度也前馈出去
        if self._prev_ref is not None and dt > 0.0:
            raw = (self._z_ref - self._prev_ref) / dt
            self._dz_ref += 0.25 * (raw - self._dz_ref)
        self._prev_ref = self._z_ref
        # 偏置积分刻意做慢（ki=0.02，时间常数 50 s）：机动本身就会让 AGL 偏离
        # 设定值，"提前起爬"期间这种偏离更大；积分一快就会把它当偏置抹掉，
        # 实测 30 m 山脊上的最小间隙从 12.2 m 掉到 2.9 m。慢 50 s 的积分在
        # 60 s 级的航段上仍足以吃掉 1 m 量级的 DEM 偏置（见文档的 ki 敏感性表）。
        if agl_meas is not None:
            e = self._agl_set - float(agl_meas)
            self._trim = float(np.clip(self._trim + self.ki * e * dt,
                                       -self.trim_limit, self.trim_limit))
        vz_des = self._dz_ref + self.kz * (self._z_ref - self._z)
        vz = self.limits.limit(vz_des, self._vz, dt)
        self._vz = vz
        self._z += vz * dt
        return self._z, vz

    @property
    def trim(self) -> float:
        return self._trim

    @property
    def reference(self) -> float:
        return self._z_ref


STRATEGIES = {
    ConstantMSL.name: ConstantMSL,
    ReactiveAGL.name: ReactiveAGL,
    DemPreview.name: DemPreview,
}


def make_strategy(name: str, limits: VerticalLimits = VerticalLimits()
                  ) -> TerrainFollowStrategy:
    """按名字构造策略（``constant_msl`` / ``reactive_agl`` / ``dem_preview``）。"""
    try:
        cls = STRATEGIES[name]
    except KeyError:
        raise KeyError(
            f"未知策略 {name!r}，可选：{', '.join(sorted(STRATEGIES))}"
        ) from None
    if cls is ConstantMSL:
        return cls()
    return cls(limits)


def steady_state_lag_m(speed_mps: float, slope: float, kp: float,
                       latency_s: float) -> float:
    """纯反馈在恒坡上的稳态 AGL 滞后（米）。

    被控对象吃的是「爬升率指令」，比例环写成 ``ṙ = Kp·e``；要跟上地形变化率
    ``ḣ = v·tanθ`` 就必须有 ``e = v·tanθ/Kp``。延迟再补一段相位滞后，等效于
    在串联通道里多一个 ``τ``，故取 ``e = v·tanθ·(1/Kp + τ)``。DEM 前馈没有这一项。
    """
    return abs(float(speed_mps)) * abs(float(slope)) * (
        1.0 / float(kp) + abs(float(latency_s)))


@dataclass(frozen=True)
class LoopConfig:
    """一次仿真的完整配置（策略之外的公共部分）。"""

    agl_set_m: float = 15.0
    safety_m: float = 5.0
    length_m: float = 600.0
    speed_mps: float = 8.0
    ramp_s: float = 4.0
    limits: VerticalLimits = VerticalLimits()
    altimeter: AltimeterConfig = AltimeterConfig()
    dem: DemConfig = DemConfig()
