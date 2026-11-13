# -*- coding: utf-8 -*-
"""Bezier 曲线与 B 样条曲线核心算法。

纯数学层，不依赖 pygame，便于单元测试。
"""

from __future__ import annotations
import math
from typing import List, Sequence, Tuple

Point = Tuple[float, float]

# B 样条在右端点处的容差：基函数定义在半开区间 [u_i, u_{i+1}) 上，
# 直接把 u 取到 u_max 会让所有基函数同时为 0，结果退化成 (0, 0)。
# 因此把定义域右端向内收一个极小量，使 clamped 样条在末点收敛到末控制点。
_SPLINE_EPS = 1e-12


# ---------------------------------------------------------------------------
# 组合数
# ---------------------------------------------------------------------------
def binomial(n: int, k: int) -> int:
    """计算 C(n, k)。"""
    if k < 0 or k > n:
        return 0
    if k == 0 or k == n:
        return 1
    k = min(k, n - k)
    result = 1
    for i in range(k):
        result = result * (n - i) // (i + 1)
    return result


# ---------------------------------------------------------------------------
# Bezier 曲线
# ---------------------------------------------------------------------------
def bezier_point_de_casteljau(points: Sequence[Point], t: float) -> Point:
    """用 de Casteljau 递推算法计算 Bezier 曲线上参数 t 处的点。

    Parameters
    ----------
    points : 控制点序列 [(x, y), ...]
    t : 参数，[0, 1]

    Returns
    -------
    (x, y) 曲线点
    """
    if not points:
        raise ValueError("控制点序列不能为空")
    pts = [list(p) for p in points]
    n = len(pts)
    for r in range(1, n):
        for i in range(n - r):
            pts[i][0] = (1 - t) * pts[i][0] + t * pts[i + 1][0]
            pts[i][1] = (1 - t) * pts[i][1] + t * pts[i + 1][1]
    return (pts[0][0], pts[0][1])


def bezier_point_bernstein(points: Sequence[Point], t: float) -> Point:
    """用 Bernstein 多项式形式计算 Bezier 曲线点。

    B(t) = Σ C(n,i) * (1-t)^(n-i) * t^i * P_i
    """
    if not points:
        raise ValueError("控制点序列不能为空")
    n = len(points) - 1
    x, y = 0.0, 0.0
    for i, (px, py) in enumerate(points):
        b = binomial(n, i) * ((1 - t) ** (n - i)) * (t ** i)
        x += b * px
        y += b * py
    return (x, y)


def bezier_derivative(points: Sequence[Point], t: float, order: int = 1) -> Point:
    """计算 Bezier 曲线的 order 阶导数在 t 处的值。

    一阶导数是切向量，二阶导数用于曲率计算。
    """
    if order == 0:
        return bezier_point_de_casteljau(points, t)
    n = len(points) - 1
    if n < order:
        return (0.0, 0.0)
    # 导数的控制点 = 原控制点差分 * n
    deriv_pts = points
    for _ in range(order):
        m = len(deriv_pts) - 1
        deriv_pts = [
            ((deriv_pts[i + 1][0] - deriv_pts[i][0]) * m,
             (deriv_pts[i + 1][1] - deriv_pts[i][1]) * m)
            for i in range(m)
        ]
    return bezier_point_de_casteljau(deriv_pts, t)


def curvature(points: Sequence[Point], t: float) -> float:
    """计算 Bezier 曲线在 t 处的曲率 κ = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)。"""
    dx, dy = bezier_derivative(points, t, 1)
    ddx, ddy = bezier_derivative(points, t, 2)
    denom = (dx * dx + dy * dy) ** 1.5
    if denom < 1e-12:
        return 0.0
    return abs(dx * ddy - dy * ddx) / denom


def arc_length(points: Sequence[Point], samples: int = 200) -> float:
    """数值积分计算 Bezier 曲线弧长（梯形法）。"""
    if len(points) < 2:
        return 0.0
    prev = bezier_point_de_casteljau(points, 0.0)
    total = 0.0
    for i in range(1, samples + 1):
        t = i / samples
        cur = bezier_point_de_casteljau(points, t)
        total += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
        prev = cur
    return total


def sample_curve(points: Sequence[Point], n: int = 200) -> List[Point]:
    """均匀采样 n 个点。"""
    return [bezier_point_de_casteljau(points, i / max(n - 1, 1)) for i in range(n)]


# ---------------------------------------------------------------------------
# 3 次 B 样条（对比用）
# ---------------------------------------------------------------------------
def bspline_basis(i: int, p: int, u: float, knots: Sequence[float]) -> float:
    """Cox-de Boor 递归计算 B 样条基函数 N_{i,p}(u)。"""
    if p == 0:
        return 1.0 if (knots[i] <= u < knots[i + 1]) else 0.0
    left, right = 0.0, 0.0
    d1 = knots[i + p] - knots[i]
    d2 = knots[i + p + 1] - knots[i + 1]
    if d1 > 1e-12:
        left = (u - knots[i]) / d1 * bspline_basis(i, p - 1, u, knots)
    if d2 > 1e-12:
        right = (knots[i + p + 1] - u) / d2 * bspline_basis(i + 1, p - 1, u, knots)
    return left + right


def _clamp_domain(u: float, degree: int, knots: Sequence[float]) -> float:
    """把参数 u 夹到 B 样条的有效定义域 [knots[degree], knots[-degree-1])。

    右端取开区间：直接取 u_max 时所有基函数都为 0，返回值会退化成 (0, 0)，
    这是「B 样条末端画到画布原点」的根因。
    """
    u_lo = knots[degree]
    u_hi = knots[len(knots) - degree - 1]
    if u_hi <= u_lo:
        return u_lo
    if u <= u_lo:
        return u_lo
    if u >= u_hi:
        return u_hi - _SPLINE_EPS * (u_hi - u_lo)
    return u


def bspline_point(points: Sequence[Point], u: float,
                  degree: int = 3, knots: Sequence[float] = None) -> Point:
    """计算 B 样条曲线上参数 u 处的点。

    Parameters
    ----------
    points : 控制点
    u : 参数；内部会夹到有效定义域 [knots[degree], knots[-degree-1]) 内
    degree : 次数（默认 3）
    knots : 节点向量；None 时自动生成 clamped 均匀节点向量

    Notes
    -----
    定义域右端是开区间（基函数支撑为 [u_i, u_{i+1})），若原样传入 u_max
    所有基函数都为 0、返回 (0, 0)。这里把 u 收进右端内侧一个极小量，
    使 clamped 样条在末点收敛到最后一个控制点。
    """
    n = len(points) - 1
    if degree > n:
        degree = n
    if knots is None:
        # clamped 均匀节点向量：两端各 degree+1 个重复，中间均匀递增
        n_ctrl = len(points)
        n_inner = n_ctrl - degree - 1  # 内部节点数
        if n_inner <= 0:
            knots = [0.0] * (degree + 1) + [1.0] * (degree + 1)
        else:
            knots = ([0.0] * (degree + 1)
                     + [float(i) for i in range(1, n_inner + 1)]
                     + [float(n_inner + 1)] * (degree + 1))
    u = _clamp_domain(u, degree, knots)
    x, y = 0.0, 0.0
    for i, (px, py) in enumerate(points):
        b = bspline_basis(i, degree, u, knots)
        x += b * px
        y += b * py
    return (x, y)


def sample_bspline(points: Sequence[Point], degree: int = 3,
                   n: int = 200) -> List[Point]:
    """均匀采样 B 样条曲线。

    有效参数范围 [knots[degree], knots[-degree-1])；末样本按 clamped 性质收敛到
    最后一个控制点（不会因半开区间退化到原点）。
    """
    if len(points) <= degree:
        return [points[0]] if points else []
    n_ctrl = len(points)
    n_inner = n_ctrl - degree - 1
    if n_inner <= 0:
        u_min, u_max = 0.0, 1.0
    else:
        u_min, u_max = 0.0, float(n_inner + 1)
    return [bspline_point(points, u_min + (u_max - u_min) * i / max(n - 1, 1), degree)
            for i in range(n)]
