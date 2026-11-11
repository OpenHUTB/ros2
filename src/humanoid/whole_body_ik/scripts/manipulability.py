# -*- coding: utf-8 -*-
"""可操作性度量与椭球分析（Yoshikawa 指标）。

可操作性
--------
对雅可比 J，定义

    w = sqrt(det(J Jᵀ))

几何意义：**末端在各方向上能产生多快速度**的椭球体积（差一个常数因子）。
w = 0 表示到了奇异位形——某个方向再怎么动关节，末端也动不了。

椭球
----
J Jᵀ 的特征分解给出椭球的半轴：

    半轴方向 = 特征向量
    半轴长度 = sqrt(特征值) = 奇异值

所以最小奇异值就是椭球最短半轴，它趋近 0 时机构"快失去一个方向"。
"""

from __future__ import annotations

import math

from linalg import det, mat_mul, symmetric_eigen, transpose

__all__ = [
    "manipulability",
    "singular_values",
    "condition_number",
    "singularity_distance",
    "manipulability_ellipsoid",
    "is_singular",
    "joint_limit_cost",
]


def _jjt(j, rows=None):
    if rows is not None:
        j = [j[r] for r in rows]
    return mat_mul(j, transpose(j))


def manipulability(j, rows=None):
    """Yoshikawa 可操作性指标 w = sqrt(det(J Jᵀ))。

    det 可能出现极小的负数（数值误差），这里取 max(0, ·) 再开方。

    关于 rows
    ---------
    平面 2R 臂的 3 维位置雅可比**恒奇异**——它本来就在平面内运动，z 方向
    永远是 0，所以 det(J Jᵀ) ≡ 0，直接算会把所有位形都判成奇异。

    这时候要只取机构真正能驱动的任务维度。平面臂传 ``rows=(0, 1)``，
    就退化成经典的 ``w = |l1·l2·sin(q1)|``。
    """
    d = det(_jjt(j, rows))
    return math.sqrt(max(0.0, d))


def singular_values(j, rows=None):
    """返回降序排列的奇异值列表。

    用 J Jᵀ 的特征值开根号得到——对 J 本身做 SVD 需要迭代算法，
    而 J Jᵀ 是对称半正定矩阵，Jacobi 法又快又稳。
    """
    vals, _vecs = symmetric_eigen(_jjt(j, rows))
    return [math.sqrt(max(0.0, v)) for v in vals]


def condition_number(j, rows=None):
    """条件数 σ_max / σ_min，衡量"接近奇异"的程度。

    σ_min = 0 时返回 inf。
    """
    sv = singular_values(j, rows)
    if not sv:
        raise ValueError("雅可比为空")
    s_max, s_min = sv[0], sv[-1]
    if s_min < 1e-15:
        return float("inf")
    return s_max / s_min


def singularity_distance(j, rows=None):
    """最小奇异值，越小越接近奇异。"""
    sv = singular_values(j, rows)
    return sv[-1] if sv else 0.0


def manipulability_ellipsoid(j, rows=None):
    """可操作性椭球。

    返回 dict：
        axes        半轴长度（降序），即奇异值
        directions  对应的单位方向（长度 = 任务维度）
        volume      椭球体积（任务维度为 3 时是 4/3πabc，2 维时是 πab）
    """
    vals, vecs = symmetric_eigen(_jjt(j, rows))
    axes = [math.sqrt(max(0.0, v)) for v in vals]
    m = len(axes)
    if m == 3:
        volume = (4.0 / 3.0) * math.pi * axes[0] * axes[1] * axes[2]
    elif m == 2:
        volume = math.pi * axes[0] * axes[1]
    else:
        volume = axes[0] if axes else 0.0
    return {
        "axes": axes,
        "directions": vecs,
        "volume": volume,
    }


def is_singular(j, threshold=1e-3, rows=None):
    """最小奇异值低于阈值就认为处于奇异位形。"""
    return singularity_distance(j, rows) < threshold


def joint_limit_cost(q, joints, margin=0.1):
    """关节限位的次级目标函数及其梯度。

    越接近限位，代价越大（双曲形式，到边界趋向无穷）。可以把返回的梯度
    投到零空间里，让冗余自由度自动躲开限位，而不影响末端位姿。

    返回 (cost, gradient)。没有限位的关节梯度为 0。
    """
    cost = 0.0
    grad = [0.0] * len(q)
    for i, (qi, joint) in enumerate(zip(q, joints)):
        if joint.limits is None:
            continue
        lo, hi = joint.limits
        span = hi - lo
        if span <= 0:
            continue
        # 归一化到 [-1, 1]，0 是中点
        mid = 0.5 * (lo + hi)
        half = 0.5 * span
        x = (qi - mid) / half
        x = max(-0.999999, min(0.999999, x))
        m = min(margin, 0.499)
        # 代价在 |x| > 1 - m 时快速上升
        edge = 1.0 - m
        if abs(x) > edge:
            d = (abs(x) - edge) / m
            cost += d * d
            # d(cost)/dq
            sign = 1.0 if x > 0 else -1.0
            grad[i] = sign * 2.0 * d / m / half
    return cost, grad
