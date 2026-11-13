"""地面平面拟合与坡度计算。

点云为 N×3 数组，坐标为机器人本体坐标系（x 前、y 左、z 上），
地面用平面方程 n·x + d = 0 表示，n 为单位法向量。

全部为纯函数，不依赖 ROS 与 PyBullet，便于单元测试与离线复算。
"""

import numpy as np


def fit_plane_least_squares(points):
    """最小二乘拟合平面，返回 (normal, d)。

    对去中心化后的点集做 SVD，最小奇异值对应的右奇异向量即法向量。
    法向量统一取 z 分量为正，避免同一平面返回两个方向。
    """
    pts = _as_points(points)
    if pts.shape[0] < 3:
        raise ValueError('至少需要 3 个点才能拟合平面')
    centroid = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - centroid, full_matrices=False)
    normal = vt[2] / np.linalg.norm(vt[2])
    if normal[2] < 0:
        normal = -normal
    return normal, -float(normal @ centroid)


def fit_plane_ransac(points, threshold=0.01, iterations=200, seed=0):
    """RANSAC 拟合平面，返回 (normal, d, inlier_ratio)。

    用于点云里混进障碍物、腿部等离群点的场景。threshold 是点到平面的
    距离阈值（米）。最后用全部内点再做一次最小二乘，精度不低于纯 RANSAC。
    """
    pts = _as_points(points)
    if pts.shape[0] < 3:
        raise ValueError('至少需要 3 个点才能拟合平面')
    rng = np.random.default_rng(seed)
    count = pts.shape[0]
    best_normal, best_d, best_ratio = None, None, 0.0
    for _ in range(iterations):
        index = rng.choice(count, size=3, replace=False)
        sample = pts[index]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-12:
            continue
        normal = normal / norm
        if normal[2] < 0:
            normal = -normal
        d = -float(normal @ sample[0])
        ratio = float(np.mean(np.abs(pts @ normal + d) < threshold))
        if ratio > best_ratio:
            best_normal, best_d, best_ratio = normal, d, ratio
    if best_normal is None:
        raise ValueError('点云退化，未能拟合出平面')
    inliers = pts[np.abs(pts @ best_normal + best_d) < threshold]
    normal, d = fit_plane_least_squares(inliers)
    return normal, d, best_ratio


def slope_angle(normal):
    """法向量与竖直方向的夹角（弧度），即地面坡度。"""
    n = np.asarray(normal, dtype=float)
    norm = np.linalg.norm(n)
    if norm < 1e-12:
        raise ValueError('法向量不能为零向量')
    return float(np.arccos(np.clip(abs(n[2]) / norm, -1.0, 1.0)))


def plane_height(normal, d, x, y):
    """平面上 (x, y) 处的高度 z；法向量接近水平时无解。"""
    n = np.asarray(normal, dtype=float)
    if abs(n[2]) < 1e-9:
        raise ValueError('地面近似垂直，无法给出高度')
    return float(-(n[0] * x + n[1] * y + d) / n[2])


def inlier_ratio(points, normal, d, threshold):
    """落在平面容差内的点占比，用于判断拟合是否被离群点污染。"""
    pts = _as_points(points)
    return float(np.mean(np.abs(pts @ np.asarray(normal, float) + d)
                         < threshold))


def _as_points(points):
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError('点云必须是 N×3 数组')
    if pts.shape[0] == 0:
        raise ValueError('点云为空')
    if not np.all(np.isfinite(pts)):
        raise ValueError('点云含 NaN 或 Inf')
    return pts
