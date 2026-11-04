"""点云特征工程（纯计算，不依赖 CARLA）。

输入是激光雷达在**传感器坐标系**下的点 ``(N, 3)``，输出是逐点特征 ``(N, F)``。
特征全部由单帧点云本身算出，不用外部地图、不用时序，因此可以离线复现。

特征顺序（共 9 维）：

====  ==========================  ============================================
序号  名称                        含义
====  ==========================  ============================================
0     x                           车体系纵向（前为正），m
1     y                           车体系横向（左为正），m
2     z                           车体系高度，m
3     r                           水平距离 sqrt(x^2+y^2)，m
4     sin_azimuth                 方位角正弦，避免 ±pi 处的跳变
5     cos_azimuth                 方位角余弦
6     elevation                   俯仰角 atan2(z, r)，rad
7     height_above_ground         离地高度，用低分位估计的路面高度扣掉
8     log_density                 邻域点数的 log(1+n)，反映"这里是墙面还是空旷路面"
====  ==========================  ============================================
"""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

import numpy as np

FEATURE_NAMES = (
    'x', 'y', 'z', 'r', 'sin_azimuth', 'cos_azimuth',
    'elevation', 'height_above_ground', 'log_density',
)


def estimate_ground_z(points: np.ndarray, percentile: float = 5.0,
                      min_points: int = 50) -> float:
    """用 z 的低分位数估计路面高度。

    激光雷达扫到的路面点通常是最低的一层，取 5% 分位比取最小值稳（避免单个噪声点）。
    点太少时退回 0.0，并由调用方在日志里提示。
    """
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[0] < min_points:
        return 0.0
    return float(np.percentile(pts[:, 2], percentile))


def voxel_keys(points: np.ndarray, voxel: float = 0.5) -> np.ndarray:
    """把点映射到体素格子的整数坐标，返回 ``(N, 3) int``。"""
    if voxel <= 0.0:
        raise ValueError('体素边长必须为正，当前 %.3f' % voxel)
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError('点云形状应为 (N, 3)，实际 %s' % (pts.shape,))
    return np.floor(pts / voxel).astype(np.int64)


def neighbor_counts(points: np.ndarray, voxel: float = 0.5) -> np.ndarray:
    """每个点所在体素里的点数（含自己），用来替代昂贵的 kNN 密度。"""
    keys = voxel_keys(points, voxel)
    counts: Dict[Tuple[int, int, int], int] = {}
    for k in keys:
        key = (int(k[0]), int(k[1]), int(k[2]))
        counts[key] = counts.get(key, 0) + 1
    return np.array([counts[(int(k[0]), int(k[1]), int(k[2]))] for k in keys], dtype=float)


def point_features(points: np.ndarray, ground_z: float | None = None,
                   voxel: float = 0.5) -> np.ndarray:
    """把 ``(N, 3)`` 点云转成 ``(N, 9)`` 特征矩阵。"""
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError('点云形状应为 (N, 3)，实际 %s' % (pts.shape,))
    if pts.shape[0] == 0:
        return np.zeros((0, len(FEATURE_NAMES)), dtype=float)

    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r = np.sqrt(x * x + y * y)
    with np.errstate(divide='ignore', invalid='ignore'):
        azimuth = np.where(r > 1e-6, np.arctan2(y, x), 0.0)
        elevation = np.where(r > 1e-6, np.arctan2(z, r), 0.0)

    if ground_z is None:
        ground_z = estimate_ground_z(pts)
    height = z - float(ground_z)

    density = neighbor_counts(pts, voxel)

    return np.column_stack([
        x, y, z, r,
        np.sin(azimuth), np.cos(azimuth),
        elevation,
        height,
        np.log1p(density),
    ])


def standardize_fit(features: np.ndarray):
    """算每维的均值与标准差，标准差为 0 的维度按 1 处理，避免除零。"""
    feats = np.asarray(features, dtype=float)
    if feats.ndim != 2 or feats.shape[0] == 0:
        raise ValueError('特征矩阵不能为空')
    mean = feats.mean(axis=0)
    std = feats.std(axis=0)
    std = np.where(std < 1e-9, 1.0, std)
    return mean, std


def standardize_apply(features: np.ndarray, mean: np.ndarray, std: np.ndarray) -> np.ndarray:
    """按训练集的均值/方差做标准化。"""
    feats = np.asarray(features, dtype=float)
    mean = np.asarray(mean, dtype=float)
    std = np.asarray(std, dtype=float)
    if feats.shape[1] != mean.shape[0] or feats.shape[1] != std.shape[0]:
        raise ValueError('特征维度与统计量不一致：%d vs %d' % (feats.shape[1], mean.shape[0]))
    return (feats - mean) / std


def voxel_downsample(points: np.ndarray, labels: np.ndarray | None = None,
                     voxel: float = 0.3):
    """每个体素保留一个点（第一次出现的），返回 ``(points, labels)``。"""
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] == 0:
        return pts.reshape(0, 3), (np.zeros(0, dtype=int) if labels is None
                                   else np.asarray(labels, dtype=int)[:0])
    keys = voxel_keys(pts, voxel)
    seen = set()
    keep: List[int] = []
    for i, k in enumerate(keys):
        key = (int(k[0]), int(k[1]), int(k[2]))
        if key not in seen:
            seen.add(key)
            keep.append(i)
    kept_points = pts[keep]
    if labels is None:
        return kept_points, None
    return kept_points, np.asarray(labels, dtype=int)[keep]


def summarize(features: np.ndarray) -> Dict[str, float]:
    """给数据体检用的简单统计：每维均值、极值，便于发现异常。"""
    feats = np.asarray(features, dtype=float)
    out: Dict[str, float] = {'n': float(feats.shape[0])}
    for i, name in enumerate(FEATURE_NAMES):
        col = feats[:, i]
        out[name + '_mean'] = float(col.mean())
        out[name + '_min'] = float(col.min())
        out[name + '_max'] = float(col.max())
    return out