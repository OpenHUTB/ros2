"""纵向制动动力学：把摩擦系数换算成减速度与停车距离。

这里只保留与热衰退直接相关的部分：车轮不抱死、制动力平均分配到四个轮，因此
整车减速度与摩擦系数成正比。常温下的基准减速度由 :attr:`BrakingSpec.reference_decel_mps2`
给出，任意盘温下的减速度为

.. math::

    a(T) = a_{ref} \\frac{\\mu(T)}{\\mu_{cold}}

单次制动按"盘温在这一次制动中不变"计算——单次制动只让盘温升高几十 K（见 README
的量级估算），连续制动的温度累积由 ``scenario.py`` 负责。
"""

from __future__ import annotations

from dataclasses import dataclass

from .disc import FrictionFadeCurve


@dataclass(frozen=True)
class BrakingSpec:
    """整车参数。"""

    mass_kg: float = 1500.0
    reference_decel_mps2: float = 8.0
    brake_count: int = 4
    gravity: float = 9.80665

    def validate(self) -> None:
        if not self.mass_kg > 0.0:
            raise ValueError("mass_kg must be positive")
        if not self.reference_decel_mps2 > 0.0:
            raise ValueError("reference_decel_mps2 must be positive")
        if self.brake_count < 1:
            raise ValueError("brake_count must be at least 1")


def deceleration_mps2(
    temperature_c: float,
    braking: BrakingSpec,
    curve: FrictionFadeCurve,
) -> float:
    """给定盘温下的可用制动减速度（m/s²）。"""
    braking.validate()
    return float(braking.reference_decel_mps2 * curve.relative(temperature_c))


def stopping_distance_m(
    speed_mps: float,
    temperature_c: float,
    braking: BrakingSpec,
    curve: FrictionFadeCurve,
) -> float:
    """定温假设下的最短停车距离 ``v² / (2 a)``（m）。"""
    if speed_mps < 0.0:
        raise ValueError("speed_mps must not be negative")
    deceleration = deceleration_mps2(temperature_c, braking, curve)
    if deceleration <= 0.0:
        return float("inf")
    return float(speed_mps**2 / (2.0 * deceleration))


def stop_energy_j(speed_mps: float, braking: BrakingSpec) -> float:
    """从 ``speed_mps`` 刹停需要耗散的动能（J），全部转化为制动器热量。"""
    if speed_mps < 0.0:
        raise ValueError("speed_mps must not be negative")
    return float(0.5 * braking.mass_kg * speed_mps**2)


def per_disc_power_w(
    speed_mps: float,
    grade: float,
    braking: BrakingSpec,
) -> float:
    """恒速下坡时单只制动盘需要耗散的功率（W）。

    坡度用小数表示（0.07 即 7%），恒速下坡时重力沿坡向分量全部由制动器平衡。
    """
    if speed_mps < 0.0:
        raise ValueError("speed_mps must not be negative")
    if not 0.0 <= grade < 1.0:
        raise ValueError("grade must be in [0, 1)")
    braking.validate()
    total = braking.mass_kg * braking.gravity * grade * speed_mps
    return float(total / braking.brake_count)
