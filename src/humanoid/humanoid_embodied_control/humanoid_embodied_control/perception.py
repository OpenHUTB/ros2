# -*- coding: utf-8 -*-
"""感知层：把传感器原始数据变成"能喂给决策的少量标量"。

课程要求里"感知"不是传感器转发，而是**用传感器数据理解环境**。这里的三个
纯函数就是"理解"的定义：

* :func:`lidar_sectors`       —— 一帧距离扫描 → N 个扇区的可通行性 [0,1]
* :func:`detect_obstacles`    —— 可通行性 → 障碍段（方位 + 最近距离）
* :func:`free_direction`      —— 可通行性 → 最空旷方位（决策层的"绕行方向"）

外加 :func:`depth_free_ratio` 与 :func:`sensor_fusion`：深度相机的
"前方可通行比例"与激光扇区做加权融合，弥补 1 线激光没有垂直视场的缺口。

设计要点（踩过的坑写在函数注释里）：
*  每扇取**最小值**而不是均值。均值会被远处空旷点稀释，导致近处障碍
   被"平均"掉——这是最典型的"看起来有感知、实际却撞上去"。
*  障碍必须按**距离排序**后才进入决策，否则状态机可能先处理远处的墙。
*  目标方位用相对角（本体系），跨 ±π 时由 geometry.wrap_pi 保证连续。
"""

from __future__ import annotations

import math

from .geometry import wrap_pi


def lidar_sectors(ranges, angle_span: float, ranges_max: float, n_sectors: int = 16):
    """把一帧测距扫描切成 ``n_sectors`` 个扇区，返回每扇的可通行性 [0,1]。

    参数
    ----
    ranges : 序列[float]
        与 ``angle_min ... angle_max`` 均匀对应的距离值；``nan`` /
        ``inf`` / 小于 0 表示无回波（视为"该方向空旷"）。
    angle_span : float
        整帧扫描覆盖的角度（弧度），例如 2π。
    ranges_max : float
        传感器最大量程，用来归一化。
    n_sectors : int
        扇区数；取 16 是"够细但不至于把整帧噪声放大"的经验值。

    返回
    ----
    list[float]
        长度 ``n_sectors``，值域 [0,1]，1 表示该方向整段都空旷。
    """
    if n_sectors <= 0:
        raise ValueError("n_sectors 必须为正")
    if angle_span <= 0.0 or ranges_max <= 0.0:
        raise ValueError("angle_span 与 ranges_max 必须为正")

    best = [ranges_max] * n_sectors
    n = len(ranges)
    if n == 0:
        return [1.0] * n_sectors

    for i, r in enumerate(ranges):
        # 无回波 → 该方向空旷，不参与最小值统计
        if r is None or (isinstance(r, float) and (math.isnan(r) or math.isinf(r))) or r < 0.0:
            continue
        idx = int(i * n_sectors / n)
        if idx >= n_sectors:
            idx = n_sectors - 1
        if r < best[idx]:
            best[idx] = float(r)

    return [max(0.0, min(1.0, d / ranges_max)) for d in best]


def sector_to_angle(index: int, n_sectors: int, angle_span: float, angle_min: float = 0.0) -> float:
    """扇区下标 → 该扇区中心角（弧度，含 ``angle_min`` 偏置）。"""
    if n_sectors <= 0:
        raise ValueError("n_sectors 必须为正")
    return angle_min + angle_span * (index + 0.5) / n_sectors


def detect_obstacles(sectors, angle_span: float, dist_max: float, threshold: float = 0.55,
                     angle_min: float = 0.0):
    """把可通行性扇区转成障碍段列表，并按距离从近到远排序。

    相邻且都低于 ``threshold`` 的扇区会合并成一个连续障碍段——合并是必须的：
    一堵墙在扫描里会跨好几个扇区，不合并的后果是决策层反复"绕过一个点"，
    实际却一直朝着同一堵墙走。

    返回
    ----
    list[dict]
        每项 ``{"angle": 段中心角, "distance": 段内最近距离(m), "width": 扇区数}``。
    """
    if not sectors:
        return []

    runs = []
    start = None
    for i, s in enumerate(sectors):
        blocked = s < threshold
        if blocked and start is None:
            start = i
        elif not blocked and start is not None:
            runs.append((start, i - 1))
            start = None
    if start is not None:
        runs.append((start, len(sectors) - 1))

    obstacles = []
    for lo, hi in runs:
        mid = 0.5 * (lo + hi) + 0.5
        angle = angle_min + angle_span * mid / len(sectors)
        distance = min(sectors[lo:hi + 1]) * dist_max
        obstacles.append({
            "angle": wrap_pi(angle),
            "distance": float(distance),
            "width": hi - lo + 1,
        })

    obstacles.sort(key=lambda o: o["distance"])
    return obstacles


def free_direction(sectors, angle_span: float, angle_min: float = 0.0) -> float:
    """返回最空旷扇区的中心角（弧度）。扇区全为空时返回 0（正前方）。

    相等可通行性时取**离正前方最近**的那一扇：否则两侧同样空旷时，
    决策层会因为浮点数微小差异随机地左右抽搐。
    """
    if not sectors:
        return 0.0

    best_i = 0
    best_score = -1.0
    best_front = float("inf")
    for i, s in enumerate(sectors):
        a = wrap_pi(sector_to_angle(i, len(sectors), angle_span, angle_min))
        front = abs(a)
        if s > best_score + 1e-9 or (abs(s - best_score) <= 1e-9 and front < best_front):
            best_i, best_score, best_front = i, s, front
    return wrap_pi(sector_to_angle(best_i, len(sectors), angle_span, angle_min))


def depth_free_ratio(depth_rows, valid_max: float):
    """由深度图（或一行深度值）估计"正前方可通行比例" [0,1]。

    ``depth_rows`` 可以是二维（行×列）也可以是一维；取中间若干行的
    **最小**深度，因为障碍最先出现在离机器人最近的一行上。
    返回值 1 表示整个 ROI 都在 ``valid_max`` 之外（空旷），0 表示贴脸。
    """
    rows = depth_rows if isinstance(depth_rows, (list, tuple)) else [depth_rows]
    if not rows:
        return 1.0
    if valid_max <= 0.0:
        raise ValueError("valid_max 必须为正")

    near = float("inf")
    for row in rows:
        for d in row:
            d = float(d)
            if d != d or math.isinf(d) or d <= 0.0:  # nan / inf / 无效
                continue
            if d < near:
                near = d
    if near == float("inf"):
        return 1.0
    return max(0.0, min(1.0, near / valid_max))


def sensor_fusion(lidar_free: float, depth_free: float, w_lidar: float = 0.7) -> float:
    """激光扇区可通行性与深度"空旷比例"的加权融合。

    为什么不取最大值：取最大会让任一传感器为空旷时整体判定为可通行，
    而实际部署里恰恰是"一个传感器看不见"才要保守。加权平均则要求两者
    都支持"能走"才认为是真的能走。
    """
    w_lidar = max(0.0, min(1.0, w_lidar))
    return w_lidar * float(lidar_free) + (1.0 - w_lidar) * float(depth_free)


def sectors_mean(sectors) -> float:
    """扇区可通行性的均值（供 :func:`sensor_fusion` 与激光做融合）。"""
    sectors = list(sectors)
    if not sectors:
        return 1.0
    return sum(float(s) for s in sectors) / len(sectors)


def front_clearance(sectors, angle_span: float, dist_max: float, half_width_sectors: int = 1):
    """正前方（含左右各 ``half_width_sectors`` 个扇区）的最小距离（米）。"""
    n = len(sectors)
    if n == 0:
        return dist_max
    centre = n // 2
    idxs = [(centre + k) % n for k in range(-half_width_sectors, half_width_sectors + 1)]
    return float(min(sectors[i] for i in idxs)) * dist_max
