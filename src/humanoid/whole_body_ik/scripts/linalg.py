# -*- coding: utf-8 -*-
"""纯标准库的小型线性代数工具。

逆运动学只需要这几种运算：

* 3x3 旋转（Rodrigues 公式）、4x4 位姿、矩阵乘法
* 解线性方程组 ``A x = b``（列主元高斯消元）
* 对称矩阵的特征分解（Jacobi 旋转法）——用来算可操作性椭球和最小奇异值

之所以不引 numpy：这个模块要能在任何装了 Python 3.8 的机器上直接跑，
和你之前那几个模块保持一致，评审 clone 下来不用装额外东西。
"""

from __future__ import annotations

import math

__all__ = [
    "mat_mul", "mat_vec", "transpose", "identity", "zeros",
    "norm", "cross", "dot", "normalize",
    "rot_x", "rot_y", "rot_z", "rodrigues", "mat3_mul", "mat3_vec",
    "compose", "inverse_rigid",
    "solve", "det", "symmetric_eigen",
]


# ---------------------------------------------------------------- 基础
def zeros(n, m=None):
    """n×m 零矩阵（m 缺省时返回长度 n 的零向量）。"""
    if m is None:
        return [0.0] * n
    return [[0.0] * m for _ in range(n)]


def identity(n):
    m = zeros(n, n)
    for i in range(n):
        m[i][i] = 1.0
    return m


def mat_mul(a, b):
    """普通矩阵乘法，a 是 p×q，b 是 q×r。"""
    p = len(a)
    q = len(b)
    r = len(b[0])
    out = zeros(p, r)
    for i in range(p):
        ai = a[i]
        oi = out[i]
        for k in range(q):
            aik = ai[k]
            if aik == 0.0:
                continue
            bk = b[k]
            for j in range(r):
                oi[j] += aik * bk[j]
    return out


def mat_vec(a, v):
    """矩阵乘向量。"""
    return [sum(a[i][k] * v[k] for k in range(len(v))) for i in range(len(a))]


def transpose(a):
    return [list(row) for row in zip(*a)]


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def normalize(v):
    n = norm(v)
    if n < 1e-12:
        raise ValueError("零向量无法归一化")
    return [x / n for x in v]


# ---------------------------------------------------------------- 旋转
def rot_x(t):
    c, s = math.cos(t), math.sin(t)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def rodrigues(axis, angle):
    """绕任意单位轴的旋转矩阵（Rodrigues 公式）。

        R = I + sinθ·[k]× + (1−cosθ)·[k]×²
    """
    k = normalize(axis)
    kx, ky, kz = k
    c = math.cos(angle)
    s = math.sin(angle)
    v = 1.0 - c
    return [
        [c + kx * kx * v, kx * ky * v - kz * s, kx * kz * v + ky * s],
        [ky * kx * v + kz * s, c + ky * ky * v, ky * kz * v - kx * s],
        [kz * kx * v - ky * s, kz * ky * v + kx * s, c + kz * kz * v],
    ]


def mat3_mul(a, b):
    return mat_mul(a, b)


def mat3_vec(a, v):
    return mat_vec(a, v)


# ---------------------------------------------------------------- 位姿
def compose(rot, pos):
    """把旋转和位移拼成 4x4 齐次变换。"""
    t = zeros(4, 4)
    for i in range(3):
        for j in range(3):
            t[i][j] = rot[i][j]
        t[i][3] = pos[i]
    t[3][3] = 1.0
    return t


def inverse_rigid(t):
    """刚体变换求逆：R⁻¹ = Rᵀ，位移取 −Rᵀp。"""
    r = [[t[i][j] for j in range(3)] for i in range(3)]
    p = [t[i][3] for i in range(3)]
    rt = transpose(r)
    p_new = [-x for x in mat_vec(rt, p)]
    return compose(rt, p_new)


# ---------------------------------------------------------------- 线性求解
def solve(a, b, tol=1e-12):
    """列主元高斯消元解 A x = b，A 为 n×n。

    奇异时抛 ValueError。阻尼最小二乘里 A = J·Jᵀ + λ²I，加了阻尼后
    一般可解；这个异常主要用来在 λ 太小时兜底。
    """
    n = len(a)
    if n == 0:
        raise ValueError("空矩阵")
    if len(b) != n:
        raise ValueError("维度不匹配")

    # 复制并拼上右端项
    m = [list(a[i]) + [b[i]] for i in range(n)]

    for col in range(n):
        # 选主元
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < tol:
            raise ValueError("矩阵奇异（第 %d 列主元接近 0）" % col)
        if pivot != col:
            m[col], m[pivot] = m[pivot], m[col]

        pv = m[col][col]
        for j in range(col, n + 1):
            m[col][j] /= pv

        for r in range(n):
            if r == col:
                continue
            f = m[r][col]
            if f == 0.0:
                continue
            for j in range(col, n + 1):
                m[r][j] -= f * m[col][j]

    return [m[i][n] for i in range(n)]


def det(a):
    """行列式（LU 分解，列主元）。"""
    n = len(a)
    m = [list(row) for row in a]
    result = 1.0
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-15:
            return 0.0
        if pivot != col:
            m[col], m[pivot] = m[pivot], m[col]
            result = -result
        result *= m[col][col]
        pv = m[col][col]
        for r in range(col + 1, n):
            f = m[r][col] / pv
            if f == 0.0:
                continue
            for j in range(col, n):
                m[r][j] -= f * m[col][j]
    return result


# ---------------------------------------------------------------- 特征分解
def symmetric_eigen(a, max_sweeps=60, tol=1e-12):
    """Jacobi 旋转法求实对称矩阵的特征值与特征向量。

    返回 ``(eigenvalues, eigenvectors)``，两者都按特征值**降序**排列；
    ``eigenvectors[k]`` 是第 k 个特征值对应的单位特征向量（列向量）。

    为什么需要它：可操作性椭球的半轴长是 J·Jᵀ 特征值的平方根，
    最小奇异值（用于自适应阻尼）也是。不引 numpy 的话这是最直接的算法。
    """
    n = len(a)
    m = [list(row) for row in a]
    v = identity(n)

    for _ in range(max_sweeps):
        off = 0.0
        for i in range(n):
            for j in range(i + 1, n):
                off += m[i][j] * m[i][j]
        if off < tol * tol:
            break

        for p in range(n - 1):
            for q in range(p + 1, n):
                if abs(m[p][q]) < 1e-18:
                    continue
                theta = (m[q][q] - m[p][p]) / (2.0 * m[p][q])
                t = (1.0 if theta >= 0 else -1.0) / (
                    abs(theta) + math.sqrt(theta * theta + 1.0))
                c = 1.0 / math.sqrt(t * t + 1.0)
                s = t * c

                # 更新对角与 pq
                mpp, mqq, mpq = m[p][p], m[q][q], m[p][q]
                m[p][p] = c * c * mpp - 2.0 * s * c * mpq + s * s * mqq
                m[q][q] = s * s * mpp + 2.0 * s * c * mpq + c * c * mqq
                m[p][q] = 0.0
                m[q][p] = 0.0

                for r in range(n):
                    if r == p or r == q:
                        continue
                    mrp, mrq = m[r][p], m[r][q]
                    m[r][p] = c * mrp - s * mrq
                    m[p][r] = m[r][p]
                    m[r][q] = s * mrp + c * mrq
                    m[q][r] = m[r][q]

                for r in range(n):
                    vrp, vrq = v[r][p], v[r][q]
                    v[r][p] = c * vrp - s * vrq
                    v[r][q] = s * vrp + c * vrq

    pairs = [(m[i][i], [v[r][i] for r in range(n)]) for i in range(n)]
    pairs.sort(key=lambda x: -x[0])
    return [p[0] for p in pairs], [p[1] for p in pairs]
