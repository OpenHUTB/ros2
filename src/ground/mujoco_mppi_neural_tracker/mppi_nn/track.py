"""8 字参考轨迹（Gerono 双纽线）与锥桶边界。

参考线用参数方程给出，控制器拿到的只是"当前横向误差/航向误差"这类可观测量，
不给未来路段——和真实赛道上"看到锥桶才反应"是一致的。
"""

from __future__ import annotations

import numpy as np

A = 6.0             # 8 字半宽（米）
LANE_HALF = 0.6     # 车道半宽（米），锥桶摆在边界上


def ref_point(t: float):
    """t ∈ [0, 2π) 的参考点与切向。"""
    x = A * np.cos(t)
    y = A * np.sin(t) * np.cos(t)
    dx = -A * np.sin(t)
    dy = A * (np.cos(t) ** 2 - np.sin(t) ** 2)
    n = np.hypot(dx, dy) + 1e-12
    return np.array([x, y]), np.array([dx / n, dy / n])


def ref_polyline(n: int = 600) -> np.ndarray:
    ts = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    return np.array([ref_point(t)[0] for t in ts])


def cone_positions(n: int = 600) -> np.ndarray:
    """沿参考线左右各 LANE_HALF 摆一圈锥桶。"""
    ts = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    out = []
    for t in ts:
        p, d = ref_point(t)
        left = p + np.array([-d[1], d[0]]) * LANE_HALF
        right = p - np.array([-d[1], d[0]]) * LANE_HALF
        out += [left, right]
    return np.array(out)


def nearest_index(path: np.ndarray, xy, hint: int | None = None, window: int = 80) -> int:
    """最近参考点索引；给了 hint 就只在窗口内搜（省算力，也避免 8 字自交处跳段）。"""
    if hint is None:
        d = np.linalg.norm(path - np.asarray(xy)[None, :], axis=1)
        return int(np.argmin(d))
    n = len(path)
    idx = (hint + np.arange(-window, window + 1)) % n
    d = np.linalg.norm(path[idx] - np.asarray(xy)[None, :], axis=1)
    return int(idx[int(np.argmin(d))])


def track_errors(path: np.ndarray, xy, yaw: float, idx: int) -> tuple[float, float]:
    """返回 (横向误差 e, 航向误差 ψ)。

    e > 0 表示车在参考线左侧；ψ 是参考切向与车头方向的夹角。
    """
    p = path[idx]
    nxt = path[(idx + 1) % len(path)]
    tangent = nxt - p
    tangent /= np.linalg.norm(tangent) + 1e-12
    normal = np.array([-tangent[1], tangent[0]])
    e = float(np.dot(np.asarray(xy) - p, normal))
    head = np.array([np.cos(yaw), np.sin(yaw)])
    cross = tangent[0] * head[1] - tangent[1] * head[0]
    psi = float(np.arctan2(cross, float(tangent @ head)))
    return e, psi
