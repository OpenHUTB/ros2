"""整机参数：把机翼与旋翼组合起来，并给出以高度为自变量的密度。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .aero import G, WingSpec, isa_density
from .rotor import RotorSpec

__all__ = ["VehicleSpec"]


@dataclass(frozen=True)
class VehicleSpec:
    """6.5 kg 级倾转旋翼验证机。

    推重比刻意取 ``T_hover / W ≈ 1.25``：悬停有 25% 余量，但**不足以在任意速度下
    靠旋翼硬撑**——这正是过渡走廊存在的前提。如果把 ``power_max`` 加到 3 kW，
    走廊会退化成整个象限（旋翼全程都能托住飞机），本模块就没有意义了。
    """

    mass: float = 6.5
    wing: WingSpec = field(default_factory=WingSpec)
    rotor: RotorSpec = field(default_factory=RotorSpec)
    rho0: float = 1.225
    cd_body: float = 0.0            # 机体附加阻力折算到机翼参考面积上的 CD 增量
    lift_lag: float = 0.20          # 迎角/俯仰回路等效滞后，s
    accel_limit: float = 2.0        # 纵向加速度指令限幅，m/s²
    tilt_max_deg: float = 100.0     # 倾转角上限（>90° 表示可产生反向推力，用于减速过渡）
    vz_limit: float = 1.5           # 升降速度指令限幅，m/s

    @property
    def weight(self) -> float:
        return self.mass * G

    @property
    def thrust_to_weight(self) -> float:
        return self.rotor.hover_thrust_max(self.rho0) / self.weight

    def density(self, alt_m: float) -> float:
        return isa_density(alt_m)

    def stall_speed(self, alt_m: float = 0.0) -> float:
        return self.wing.stall_speed(self.weight, self.density(alt_m))

    def hover_speed_limit(self, alt_m: float = 0.0) -> float:
        """旋翼单独能吊住全重的最快平飞速度（前进比限制）。"""
        rho = self.density(alt_m)
        w = self.weight
        t_static = self.rotor.hover_thrust_max(rho)
        if t_static <= w:
            return 0.0
        return self.rotor.v_zero_thrust * (1.0 - w / t_static)

    def min_drag_speed(self, alt_m: float = 0.0) -> float:
        return self.wing.min_drag_speed(self.weight, self.density(alt_m))

    def drag_at(self, v: float, cl: float, rho: float) -> float:
        return 0.5 * rho * v * v * self.wing.area * (self.wing.cd(cl) + self.cd_body)

    def lift_at(self, v: float, cl: float, rho: float) -> float:
        return 0.5 * rho * v * v * self.wing.area * cl

    def cl_for_lift(self, lift: float, v: float, rho: float) -> float:
        q = 0.5 * rho * v * v * self.wing.area
        if q <= 0:
            return math.copysign(math.inf, lift) if lift else 0.0
        return lift / q
