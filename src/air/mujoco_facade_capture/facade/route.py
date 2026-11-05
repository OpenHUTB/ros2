"""根据相机视场角和重叠率生成矩形立面蛇形扫描路径。"""

from __future__ import annotations

import math
import numpy as np


def scan_route(width: float = 6.0, height: float = 3.0, distance: float = 4.0,
               overlap: float = .55) -> np.ndarray:
    """返回世界坐标航点；墙面位于 y=0，无人机朝 +y 拍摄。"""
    if not all(math.isfinite(v) and v > 0 for v in (width, height, distance)):
        raise ValueError("扫描宽、高及壁距必须是正的有限数")
    if not 0 <= overlap < .95:
        raise ValueError("重叠率应位于 [0, 0.95)")
    footprint_h = 2 * distance * math.tan(math.radians(25))
    footprint_w = footprint_h * 4 / 3
    nx = max(2, math.ceil(width / (footprint_w * (1 - overlap))) + 1)
    nz = max(2, math.ceil(height / (footprint_h * (1 - overlap))) + 1)
    xs = np.linspace(-width / 2, width / 2, nx)
    zs = np.linspace(1.5, 1.5 + height, nz)
    return np.array([(x, -distance, z) for row, z in enumerate(zs)
                     for x in (xs if row % 2 == 0 else xs[::-1])], dtype=float)
