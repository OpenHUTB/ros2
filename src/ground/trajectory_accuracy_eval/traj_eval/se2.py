#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""二维位姿（SE2）与轨迹对齐。

纯 NumPy 实现，不依赖 ROS，便于离线单元测试。

约定
----
* 位姿用 ``Pose(x, y, yaw)`` 表示，``yaw`` 为绕 z 轴角度（rad，逆时针为正）；
* 复合 ``compose(a, b)`` 表示"先 a 后 b"，即 ``T_a @ T_b``；
* 平移统一用 2x2 旋转矩阵 ``R`` 与 2 维平移向量 ``t`` 表示，不用齐次矩阵，
  避免小问题里引入 3x3 矩阵运算的开销。
"""

from __future__ import annotations

import math
from typing import List, NamedTuple, Sequence, Tuple

import numpy as np


class Pose(NamedTuple):
    """二维位姿。"""

    x: float
    y: float
    yaw: float


def wrap_angle(a: float) -> float:
    """把角度归一化到 (-pi, pi]。"""
    return float((a + math.pi) % (2.0 * math.pi) - math.pi)


def compose(a: Pose, b: Pose) -> Pose:
    """位姿复合：先 a 再 b（等价于矩阵乘 T_a @ T_b）。"""
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose(
        a.x + c * b.x - s * b.y,
        a.y + s * b.x + c * b.y,
        wrap_angle(a.yaw + b.yaw),
    )


def inverse(p: Pose) -> Pose:
    """位姿求逆。"""
    c, s = math.cos(p.yaw), math.sin(p.yaw)
    return Pose(-(c * p.x + s * p.y), -(-s * p.x + c * p.y), wrap_angle(-p.yaw))


def positions(poses: Sequence[Pose]) -> np.ndarray:
    """取出 (N, 2) 位置数组。"""
    return np.array([[p.x, p.y] for p in poses], dtype=float)


def yaws(poses: Sequence[Pose]) -> np.ndarray:
    """取出 (N,) 朝向数组。"""
    return np.array([p.yaw for p in poses], dtype=float)


def path_length(poses: Sequence[Pose]) -> float:
    """轨迹总长度（相邻位置欧氏距离之和）。"""
    if len(poses) < 2:
        return 0.0
    xy = positions(poses)
    return float(np.hypot(*np.diff(xy, axis=0).T).sum())


class Alignment(NamedTuple):
    """相似变换结果：dst ≈ s * R @ src + t。"""

    scale: float
    rotation: np.ndarray   # 2x2
    translation: np.ndarray  # (2,)

    @property
    def yaw_offset(self) -> float:
        """旋转矩阵对应的角度（rad）。"""
        return math.atan2(self.rotation[1, 0], self.rotation[0, 0])

    def apply_points(self, pts: np.ndarray) -> np.ndarray:
        return self.scale * (pts @ self.rotation.T) + self.translation

    def apply_yaws(self, src_yaws: np.ndarray) -> np.ndarray:
        return np.array([wrap_angle(y + self.yaw_offset) for y in src_yaws])


def align_umeyama(src: np.ndarray, dst: np.ndarray,
                  with_scale: bool = False) -> Alignment:
    """求把 ``src`` 对齐到 ``dst`` 的最优相似变换（Umeyama / Kabsch 闭式解）。

    最小化 ``sum || s R src_i + t - dst_i ||^2``。
    ``with_scale=False`` 时固定 ``s=1``（只做刚体对齐），这是 ATE 评估的常用做法：
    里程计的**尺度误差**本身就是待评估的对象，不应在对齐时被吸收掉。

    参数
    ----
    src, dst : (N, 2) 点集，N >= 2
    with_scale : 是否同时估计尺度 s

    返回
    ----
    Alignment(scale, rotation, translation)
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 2:
        raise ValueError("src 与 dst 必须是同形状的 (N, 2) 数组")
    if src.shape[0] < 2:
        raise ValueError("至少需要 2 个点才能求对齐")

    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    src_c = src - mu_s
    dst_c = dst - mu_d

    # 2x2 的协方差矩阵：S = (dst_c^T @ src_c) / N
    s_mat = (dst_c.T @ src_c) / src.shape[0]
    u, d, vt = np.linalg.svd(s_mat)

    # 修正反射（保证是纯旋转，det(R)=+1）
    sign = np.eye(2)
    if np.linalg.det(u) * np.linalg.det(vt) < 0.0:
        sign[1, 1] = -1.0
    rot = u @ sign @ vt

    if with_scale:
        var_src = float((src_c ** 2).sum() / src.shape[0])
        if var_src < 1e-12:
            scale = 1.0
        else:
            scale = float(np.trace(np.diag(d) @ sign) / var_src)
    else:
        scale = 1.0

    trans = mu_d - scale * (rot @ mu_s)
    return Alignment(scale=scale, rotation=rot, translation=trans)


def transform_poses(poses: Sequence[Pose], alignment: Alignment) -> List[Pose]:
    """对整条轨迹应用相似变换（位置与朝向都变换）。"""
    pts = alignment.apply_points(positions(poses))
    ys = alignment.apply_yaws(yaws(poses))
    return [Pose(float(x), float(y), float(yaw)) for (x, y), yaw in zip(pts, ys)]
