"""传感器数据规整（无 ROS 依赖，便于单元测试）。

节点里最容易写错、又最难在无 ROS 环境验证的就是这类"把消息字段折成标量"的胶水代码，
因此单独抽出来并把边界情况（NaN、inf、全越界、扇区内无有效值）都做成纯函数。
"""
from __future__ import annotations

import math
from typing import Iterable, Optional

FRONT_SECTOR = math.radians(12.0)   # 取前向 ±12° 扇区


def front_range(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    sector: float = FRONT_SECTOR,
    max_range: float = 30.0,
) -> Optional[float]:
    """从一帧激光测距里取前向扇区的最小值。

    无效值（NaN、inf、<=0）与扇区外的点直接跳过；扇区内没有有效值时返回 ``None``，
    让上层知道"这一帧没有可用测距"，而不是拿默认值当真实距离。
    """
    if angle_increment == 0.0:
        return None
    best: Optional[float] = None
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r <= 0.0:
            continue
        angle = angle_min + i * angle_increment
        if abs(angle) > sector:
            continue
        if best is None or r < best:
            best = r
    if best is None:
        return None
    return min(best, max_range)


def clamp_range(value: Optional[float], max_range: float = 30.0) -> Optional[float]:
    """把超出量程的读数截断，其余原样返回。"""
    if value is None or not math.isfinite(value):
        return None
    return max(0.0, min(value, max_range))
