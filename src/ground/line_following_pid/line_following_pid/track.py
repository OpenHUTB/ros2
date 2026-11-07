"""正弦赛道：中心线采样、最近点查询、切向与横向误差。"""

import math

from . import config


def build_track():
    """返回中心线采样点列表 [(x, y), ...]。"""
    pts = []
    x = config.TRACK_X0
    while x <= config.TRACK_X1:
        y = config.TRACK_BASE + config.TRACK_AMP * math.sin(config.TRACK_K * x)
        pts.append((x, y))
        x += config.TRACK_STEP
    return pts


def nearest_point(pts, p):
    """返回赛道上离 p 最近的点索引。"""
    best_i, best_d = 0, float("inf")
    for i, q in enumerate(pts):
        d = (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def cross_track_error(pts, p, heading, idx=None):
    """横向误差：正表示车辆在赛道中心线右侧（沿行驶方向）。"""
    if idx is None:
        idx = nearest_point(pts, p)
    # 用相邻点估切向
    if idx + 1 < len(pts):
        a, b = pts[idx], pts[idx + 1]
    else:
        a, b = pts[idx - 1], pts[idx]
    tx, ty = b[0] - a[0], b[1] - a[1]
    tl = math.hypot(tx, ty) or 1e-6
    tx, ty = tx / tl, ty / tl
    # 法向（右手侧）
    nx, ny = -ty, tx
    q = pts[idx]
    return (p[0] - q[0]) * nx + (p[1] - q[1]) * ny
