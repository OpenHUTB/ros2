"""制动盘的集总热模型与摩擦系数的热衰退曲线。

模型把一个制动盘当作温度均匀的集总热容：

.. math::

    C \\frac{dT}{dt} = P_{in} - hA (T - T_{amb})
      - \\varepsilon \\sigma A \\left((T+273.15)^4 - (T_{amb}+273.15)^4\\right)

其中 ``C = m c_p`` 是盘热容，``hA`` 是对流散热系数，后一项是辐射散热。摩擦系数
随盘温升高而下降（热衰退），用分段线性的 :class:`FrictionFadeCurve` 描述。

这是工程上常用的简化：忽略了盘内温度梯度、刹车片导热、轮辋对流差异和车速对
对流系数的强化，因此给出的温度偏低、衰退偏慢，具体限制见模块 README。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: 绝对零度偏移（K）
KELVIN_OFFSET = 273.15
#: 斯特藩-玻尔兹曼常数（W/(m²·K⁴)）
STEFAN_BOLTZMANN = 5.670374419e-8


@dataclass(frozen=True)
class FrictionFadeCurve:
    """摩擦系数随温度的分段线性衰退曲线。

    ``mu_cold`` 为常温摩擦系数；温度超过 ``fade_start_c`` 后线性下降到
    ``fade_end_c`` 处的 ``mu_hot``，之后保持不变（不再继续恶化，代表最坏情况）。
    """

    mu_cold: float = 0.40
    fade_start_c: float = 250.0
    fade_end_c: float = 650.0
    mu_hot: float = 0.22

    def validate(self) -> None:
        if not 0.0 < self.mu_hot < self.mu_cold:
            raise ValueError("need 0 < mu_hot < mu_cold")
        if not self.fade_start_c < self.fade_end_c:
            raise ValueError("fade_start_c must be below fade_end_c")

    def mu(self, temperature_c) -> np.ndarray:
        """给定盘温（°C）返回摩擦系数，支持标量与数组。"""
        self.validate()
        temperature = np.asarray(temperature_c, dtype=float)
        span = self.fade_end_c - self.fade_start_c
        ratio = np.clip((temperature - self.fade_start_c) / span, 0.0, 1.0)
        return self.mu_cold + (self.mu_hot - self.mu_cold) * ratio

    def relative(self, temperature_c) -> np.ndarray:
        """相对于常温摩擦系数的比例，用于换算可提供的制动减速度。"""
        return self.mu(temperature_c) / self.mu_cold


@dataclass(frozen=True)
class BrakeDiscSpec:
    """单只制动盘的热参数（默认对应常见的通风盘）。"""

    disc_mass_kg: float = 7.0
    specific_heat_j_per_kgk: float = 500.0
    cooling_w_per_k: float = 15.0
    radiation_area_m2: float = 0.06
    emissivity: float = 0.80
    ambient_c: float = 25.0
    max_temperature_c: float = 900.0

    def validate(self) -> None:
        if not self.disc_mass_kg > 0.0:
            raise ValueError("disc_mass_kg must be positive")
        if not self.specific_heat_j_per_kgk > 0.0:
            raise ValueError("specific_heat_j_per_kgk must be positive")
        if self.cooling_w_per_k < 0.0:
            raise ValueError("cooling_w_per_k must not be negative")
        if not 0.0 <= self.emissivity <= 1.0:
            raise ValueError("emissivity must be in [0, 1]")
        if not self.radiation_area_m2 >= 0.0:
            raise ValueError("radiation_area_m2 must not be negative")

    @property
    def heat_capacity_j_per_k(self) -> float:
        """盘热容 ``m c_p``（J/K）。"""
        return self.disc_mass_kg * self.specific_heat_j_per_kgk


class BrakeDisc:
    """单只制动盘：给定制动功率，积分盘温并给出当前摩擦系数。"""

    def __init__(
        self,
        spec: BrakeDiscSpec | None = None,
        curve: FrictionFadeCurve | None = None,
    ) -> None:
        self.spec = spec or BrakeDiscSpec()
        self.curve = curve or FrictionFadeCurve()
        self.spec.validate()
        self.curve.validate()

    # ------------------------------------------------------------------ 散热
    def cooling_power_w(self, temperature_c: float) -> float:
        """对流 + 辐射散热功率（W）。"""
        delta = float(temperature_c) - self.spec.ambient_c
        convective = self.spec.cooling_w_per_k * delta
        kelvin = float(temperature_c) + KELVIN_OFFSET
        ambient_kelvin = self.spec.ambient_c + KELVIN_OFFSET
        radiative = (
            self.spec.emissivity
            * STEFAN_BOLTZMANN
            * self.spec.radiation_area_m2
            * (kelvin**4 - ambient_kelvin**4)
        )
        return convective + radiative

    def temperature_rate_c_per_s(self, temperature_c: float, input_power_w: float) -> float:
        """盘温变化率（K/s）。"""
        net = float(input_power_w) - self.cooling_power_w(temperature_c)
        return net / self.spec.heat_capacity_j_per_k

    def step(self, temperature_c: float, input_power_w: float, dt: float) -> float:
        """用四阶 Runge-Kutta 推进一步（保持温度在物理范围内）。"""
        if not dt > 0.0:
            raise ValueError("dt must be positive")
        t0 = float(temperature_c)
        k1 = self.temperature_rate_c_per_s(t0, input_power_w)
        k2 = self.temperature_rate_c_per_s(t0 + 0.5 * dt * k1, input_power_w)
        k3 = self.temperature_rate_c_per_s(t0 + 0.5 * dt * k2, input_power_w)
        k4 = self.temperature_rate_c_per_s(t0 + dt * k3, input_power_w)
        updated = t0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        return float(min(max(updated, self.spec.ambient_c), self.spec.max_temperature_c))

    # ------------------------------------------------------------------ 稳态
    def equilibrium_temperature_c(
        self,
        input_power_w: float,
        low_c: float | None = None,
        high_c: float | None = None,
        tolerance_c: float = 1.0e-4,
    ) -> float | None:
        """求散热与输入功率平衡时的盘温；功率超过散热能力时返回 ``None``。"""
        power = float(input_power_w)
        low = self.spec.ambient_c if low_c is None else float(low_c)
        high = self.spec.max_temperature_c if high_c is None else float(high_c)
        if power <= 0.0:
            return self.spec.ambient_c
        if self.cooling_power_w(high) < power:
            return None
        for _ in range(200):
            middle = 0.5 * (low + high)
            if self.cooling_power_w(middle) < power:
                low = middle
            else:
                high = middle
            if high - low < tolerance_c:
                break
        return 0.5 * (low + high)

    def time_to_reach_c(
        self,
        input_power_w: float,
        target_c: float,
        dt: float = 0.5,
        limit_s: float = 3600.0,
    ) -> float | None:
        """恒功率加热从环境温度升到 ``target_c`` 所需时间；达不到返回 ``None``。"""
        target = float(target_c)
        if target <= self.spec.ambient_c:
            return 0.0
        equilibrium = self.equilibrium_temperature_c(input_power_w)
        if equilibrium is None or equilibrium < target:
            return None
        temperature = self.spec.ambient_c
        elapsed = 0.0
        while elapsed < limit_s:
            temperature = self.step(temperature, input_power_w, dt)
            elapsed += dt
            if temperature >= target:
                return elapsed
        return None
