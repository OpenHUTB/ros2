"""大气与机翼气动模型。

坐标与符号约定
--------------
* 全部使用 SI 单位。速度 ``v`` 为空速（沿航迹），迎角 ``alpha`` 为弧度。
* 升力 ``L = 0.5 * rho * v^2 * S * CL``，阻力 ``D = 0.5 * rho * v^2 * S * CD``。
* 失速后升力系数按 ``cos`` 型衰减，用于让"掉出走廊"这件事在仿真里可观测，
  而不是在 ``CL > CLmax`` 时出现数值跳变。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

G = 9.80665
RHO0 = 1.225

__all__ = ["G", "RHO0", "isa_density", "WingSpec"]


def isa_density(alt_m: float) -> float:
    """国际标准大气（对流层 0–11000 m）密度，kg/m³。"""
    if alt_m < -500.0 or alt_m > 11000.0:
        raise ValueError(f"高度超出模型范围（-500…11000 m）：{alt_m}")
    t0, lapse, r_air, g0, p0 = 288.15, 0.0065, 287.05287, 9.80665, 101325.0
    t = t0 - lapse * alt_m
    p = p0 * (t / t0) ** (g0 / (r_air * lapse))
    return p / (r_air * t)


@dataclass(frozen=True)
class WingSpec:
    """平直翼的升阻特性。默认值对应 6.5 kg 级倾转旋翼验证机。"""

    area: float = 0.55          # m^2
    cl0: float = 0.24           # 零迎角升力系数
    cl_alpha: float = 4.60      # 1/rad
    alpha_stall_deg: float = 14.0
    cd0: float = 0.030
    aspect_ratio: float = 7.40
    oswald: float = 0.78
    post_stall_decay: float = 6.0  # 失速后衰减速率（1/rad）

    def __post_init__(self) -> None:
        if self.area <= 0 or self.cl_alpha <= 0 or self.oswald <= 0:
            raise ValueError("机翼参数必须为正")

    @property
    def alpha_stall(self) -> float:
        return math.radians(self.alpha_stall_deg)

    @property
    def cl_max(self) -> float:
        """最大升力系数（出现在正失速迎角处）。"""
        return self.cl0 + self.cl_alpha * self.alpha_stall

    @property
    def cl_min(self) -> float:
        """负失速迎角处的升力系数，用于对称包线。"""
        return self.cl0 - self.cl_alpha * self.alpha_stall

    def cl(self, alpha: float) -> float:
        """升力系数，含失速后的平滑衰减。"""
        a_s = self.alpha_stall
        if -a_s <= alpha <= a_s:
            return self.cl0 + self.cl_alpha * alpha
        peak = self.cl_max if alpha > 0 else self.cl_min
        excess = abs(alpha) - a_s
        # 用 exp 而不是硬截断：失速后升力系数随迎角继续增大而衰减
        return peak * math.exp(-self.post_stall_decay * excess)

    def cd(self, cl: float) -> float:
        """诱导 + 零升阻力。"""
        k = 1.0 / (math.pi * self.oswald * self.aspect_ratio)
        return self.cd0 + k * cl * cl

    def lift(self, v: float, rho: float, alpha: float) -> float:
        return 0.5 * rho * v * v * self.area * self.cl(alpha)

    def drag(self, v: float, rho: float, alpha: float) -> float:
        return 0.5 * rho * v * v * self.area * self.cd(self.cl(alpha))

    def max_lift(self, v: float, rho: float) -> float:
        """给定空速下的最大可用升力（按 CLmax 取，不依赖迎角）。"""
        return 0.5 * rho * v * v * self.area * self.cl_max

    def stall_speed(self, weight: float, rho: float) -> float:
        """以 CLmax 平飞所需的最小速度，m/s。"""
        if weight <= 0:
            raise ValueError("重量必须为正")
        return math.sqrt(2.0 * weight / (rho * self.area * self.cl_max))

    def min_drag_speed(self, weight: float, rho: float) -> float:
        """升阻比最大处的速度（诱导阻力等于零升阻力）。"""
        k = 1.0 / (math.pi * self.oswald * self.aspect_ratio)
        cl_md = math.sqrt(self.cd0 / k)
        return math.sqrt(2.0 * weight / (rho * self.area * cl_md))
