"""模拟传感器数据计算函数."""

import math


def periodic_sensor_value(
        elapsed_sec: float,
        base_value: float,
        amplitude: float,
        period_sec: float,
) -> float:
    """计算周期变化的模拟传感器值."""
    if period_sec <= 0.0:
        raise ValueError('period_sec 必须大于 0')
    phase = 2.0 * math.pi * elapsed_sec / period_sec
    return base_value + amplitude * math.sin(phase)
