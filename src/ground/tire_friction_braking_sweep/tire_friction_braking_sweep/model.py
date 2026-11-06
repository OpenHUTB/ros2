"""轮胎附着与制动距离的两区段模型。

制动减速度受两个上限约束：

* **轮胎附着**：``a <= μ g``（轮胎能传递的最大纵向力）
* **制动器**：``a <= a_brake``（制动扭矩与整车质量决定的上限）

所以理想化的减速度是两者取小：

.. math::

    a_{\\text{ideal}}(\\mu) = \\min(\\mu g,\\; a_{\\text{brake}})

对应的最短制动距离仍是经典结果：

.. math::

    s = \\frac{v_0^2}{2a}

真实车辆达不到 ``μ g`` 的上限（滑移率、载荷转移、制动力分配都会打折），所以模块另外
用一条饱和曲线描述实测趋势：

.. math::

    a(\\mu) = \\frac{a_{max}\\,\\mu}{\\mu + \\mu_{1/2}}

它有两个参数：``a_max`` 是附着系数趋于无穷时的极限减速度（等价于制动器瓶颈），
``μ_{1/2}`` 是达到一半极限所需的附着系数。这条曲线可用 ``1/a`` 对 ``1/μ`` 线性回归求出，
不需要非线性优化。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: 重力加速度（m/s²）
GRAVITY = 9.80665


@dataclass(frozen=True)
class BrakingSpec:
    """试验条件。"""

    mass_kg: float = 1500.0
    reference_speed_kmh: float = 50.0
    brake_ratio: float = 1.0

    def validate(self) -> None:
        if not self.mass_kg > 0.0:
            raise ValueError("mass_kg must be positive")
        if not self.reference_speed_kmh > 0.0:
            raise ValueError("reference_speed_kmh must be positive")
        if not 0.0 < self.brake_ratio <= 1.0:
            raise ValueError("brake_ratio must be in (0, 1]")

    @property
    def reference_speed_mps(self) -> float:
        return float(self.reference_speed_kmh / 3.6)


def friction_bound_decel_mps2(tire_friction) -> np.ndarray:
    """附着理论上限 ``μ g``（m/s²）。"""
    friction = np.asarray(tire_friction, dtype=float)
    if np.any(friction <= 0.0):
        raise ValueError("tire_friction must be positive")
    return friction * GRAVITY


def ideal_decel_mps2(tire_friction, brake_limit_mps2: float) -> np.ndarray:
    """两区段理想模型：取附着上限与制动器上限中的较小者。"""
    if not brake_limit_mps2 > 0.0:
        raise ValueError("brake_limit_mps2 must be positive")
    return np.minimum(friction_bound_decel_mps2(tire_friction), float(brake_limit_mps2))


def saturated_decel_mps2(tire_friction, a_max_mps2: float, mu_half: float) -> np.ndarray:
    """饱和曲线 ``a = a_max μ / (μ + μ_half)``。"""
    if not a_max_mps2 > 0.0:
        raise ValueError("a_max_mps2 must be positive")
    if not mu_half > 0.0:
        raise ValueError("mu_half must be positive")
    friction = np.asarray(tire_friction, dtype=float)
    if np.any(friction <= 0.0):
        raise ValueError("tire_friction must be positive")
    return a_max_mps2 * friction / (friction + mu_half)


def braking_distance_m(speed_mps, decel_mps2) -> np.ndarray:
    """最短制动距离 ``s = v² / (2a)``（m）。"""
    speed = np.asarray(speed_mps, dtype=float)
    deceleration = np.asarray(decel_mps2, dtype=float)
    if np.any(speed < 0.0):
        raise ValueError("speed_mps must not be negative")
    if np.any(deceleration <= 0.0):
        raise ValueError("decel_mps2 must be positive")
    return np.square(speed) / (2.0 * deceleration)


def bottleneck(
    tire_friction: float,
    brake_limit_mps2: float,
    tolerance: float = 0.85,
) -> str:
    """判定给定附着系数下哪个上限起作用。

    ``μ g`` 低于 ``tolerance × 制动器上限`` 时判为附着受限，否则判为制动器受限。
    """
    if not 0.0 < tolerance < 1.0:
        raise ValueError("tolerance must be in (0, 1)")
    attachment = float(tire_friction) * GRAVITY
    if attachment < tolerance * float(brake_limit_mps2):
        return "附着受限"
    return "制动器受限"
