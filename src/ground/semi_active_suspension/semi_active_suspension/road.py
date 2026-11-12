"""路面激励：单个减速带、ISO 8608 随机路面，以及制动/转向的等效输入。

路面写成空间坐标的函数 z(x)，四个轮子按各自的纵向位置取同一张路面，
这样前后轮的"时间差"是自然产生的，不需要额外加延时。
"""
from __future__ import annotations

import math
import random

# ISO 8608 路面不平度系数 Gd(n0)，单位 m^3，n0 = 0.1 cycle/m
ISO_GD = {
    "A": 16e-6, "B": 64e-6, "C": 256e-6, "D": 1024e-6, "E": 4096e-6,
}


class Road:
    """路面高程函数，子类实现 height(x)。"""

    def height(self, x: float) -> float:
        raise NotImplementedError

    def slope(self, x: float, dx: float = 1e-3) -> float:
        return (self.height(x + dx) - self.height(x - dx)) / (2.0 * dx)


class FlatRoad(Road):
    def height(self, x: float) -> float:
        return 0.0


class BumpRoad(Road):
    """单个半正弦减速带。"""

    def __init__(self, height: float = 0.05, length: float = 0.4,
                 start: float = 2.0) -> None:
        self.height_m = height
        self.length_m = length
        self.start = start

    def height(self, x: float) -> float:
        local = x - self.start
        if local < 0.0 or local > self.length_m:
            return 0.0
        return self.height_m * math.sin(math.pi * local / self.length_m)


class IsoRoad(Road):
    """ISO 8608 随机路面，用有限项正弦叠加生成，相位固定以便复现。"""

    def __init__(self, grade: str = "C", seed: int = 20261111,
                 low: float = 0.011, high: float = 2.83,
                 samples: int = 200) -> None:
        coefficient = ISO_GD[grade]
        rng = random.Random(seed)
        n0 = 0.1
        step = (high - low) / (samples - 1)
        self.waves = []
        for k in range(samples):
            spatial_freq = low + k * step
            amplitude = math.sqrt(
                2.0 * coefficient * step * (spatial_freq / n0) ** -2.0)
            phase = rng.uniform(0.0, 2.0 * math.pi)
            self.waves.append((spatial_freq, amplitude, phase))

    def height(self, x: float) -> float:
        total = 0.0
        for spatial_freq, amplitude, phase in self.waves:
            total += amplitude * math.sin(
                2.0 * math.pi * spatial_freq * x + phase)
        return total


class BrakeRoad(FlatRoad):
    """平路 + 制动减速度，用来考察点头。"""

    def __init__(self, decel: float = 5.9) -> None:
        self.decel = decel


class CornerRoad(FlatRoad):
    """平路 + 侧向加速度，用来考察侧倾。"""

    def __init__(self, lateral: float = 3.9) -> None:
        self.lateral = lateral
