"""旋翼（定距桨）推力模型。

两条约束取最小值，二者都来自真实倾转旋翼机的已知限制：

1. **功率限制**：动量理论给出理想诱导功率 ``P_i = T * (V + v_i)``，
   ``v_i = sqrt(T / (2 * rho * A))``；除以品质因数 ``fm`` 得到轴功率，
   给定 ``P_max`` 反解 ``T``（对 ``T`` 严格单调，用二分）。
2. **前进比限制**：定距桨在某个前飞速度 ``v_zero_thrust`` 处推力归零，
   在此之前可近似为线性衰减。这条限制决定了"高速下桨叶再也托不住飞机"。

``max_thrust(V)`` 取二者较小值，因此低速端是功率约束、高速端是前进比约束 —
过渡走廊的两个边界正是由这两条约束分别生成的。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = ["RotorSpec"]


@dataclass(frozen=True)
class RotorSpec:
    n_rotors: int = 4
    diameter: float = 0.254        # 单桨直径，m（10 英寸）
    power_max: float = 1400.0      # 全机最大轴功率，W
    figure_of_merit: float = 0.72  # 悬停品质因数
    v_zero_thrust: float = 26.0    # 定距桨推力归零速度，m/s
    tilt_rate_max_deg: float = 25.0  # 倾转角速率上限，°/s
    spool_tau: float = 0.12        # 转速/推力一阶滞后，s

    def __post_init__(self) -> None:
        if self.n_rotors < 1 or self.diameter <= 0 or self.power_max <= 0:
            raise ValueError("旋翼参数非法")
        if not 0.0 < self.figure_of_merit <= 1.0:
            raise ValueError("品质因数应在 (0, 1] 内")
        if self.v_zero_thrust <= 0:
            raise ValueError("v_zero_thrust 必须为正")

    @property
    def disk_area(self) -> float:
        """全机桨盘总面积，m²。"""
        return self.n_rotors * math.pi * self.diameter ** 2 / 4.0

    @property
    def tilt_rate_max(self) -> float:
        return math.radians(self.tilt_rate_max_deg)

    # ------------------------------------------------------------------ 功率
    def induced_velocity(self, thrust: float, rho: float) -> float:
        """动量理论诱导速度，m/s。"""
        if thrust <= 0:
            return 0.0
        return math.sqrt(max(thrust, 0.0) / (2.0 * rho * self.disk_area))

    def shaft_power(self, thrust: float, speed: float, rho: float) -> float:
        """产生 ``thrust`` 并以 ``speed`` 前飞所需的轴功率，W。"""
        if thrust <= 0:
            return 0.0
        effective = max(speed, 0.0) + self.induced_velocity(thrust, rho)
        return thrust * effective / self.figure_of_merit

    def hover_thrust_max(self, rho: float) -> float:
        """悬停时功率限制下的最大推力（闭式解 ``T = (P*fm*sqrt(2*rho*A))^(2/3)``）。"""
        k = self.power_max * self.figure_of_merit * math.sqrt(2.0 * rho * self.disk_area)
        return k ** (2.0 / 3.0)

    def power_limited_thrust(self, speed: float, rho: float, tol: float = 1e-4) -> float:
        """定功率下的最大推力：对 ``T`` 二分求解 ``shaft_power(T) = P_max``。"""
        lo, hi = 0.0, self.hover_thrust_max(rho)
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if self.shaft_power(mid, speed, rho) <= self.power_max:
                lo = mid
            else:
                hi = mid
            if hi - lo < tol:
                break
        return lo

    def advance_ratio_thrust(self, speed: float, rho: float) -> float:
        """定距桨前进比限制：线性衰减到 ``v_zero_thrust`` 处归零。"""
        ratio = 1.0 - max(speed, 0.0) / self.v_zero_thrust
        return self.hover_thrust_max(rho) * max(ratio, 0.0)

    def max_thrust(self, speed: float, rho: float) -> float:
        """当前空速下的最大可用推力（两条限制取最小）。"""
        return min(self.power_limited_thrust(speed, rho),
                   self.advance_ratio_thrust(speed, rho))

    def hover_power(self, weight: float, rho: float) -> float:
        """悬停所需轴功率，W。"""
        return self.shaft_power(weight, 0.0, rho)
