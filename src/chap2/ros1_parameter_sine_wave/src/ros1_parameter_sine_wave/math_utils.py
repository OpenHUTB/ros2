"""正弦信号计算函数."""

import math


def compute_sine(
        amplitude: float,
        frequency_hz: float,
        elapsed_sec: float,
        phase_rad: float = 0.0,
) -> float:
    """计算给定时刻的正弦信号值."""
    angular_position = 2.0 * math.pi * frequency_hz * elapsed_sec + phase_rad
    return amplitude * math.sin(angular_position)
