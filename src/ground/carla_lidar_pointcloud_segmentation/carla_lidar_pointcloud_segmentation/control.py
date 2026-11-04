"""用分割结果做轨迹跟踪与避障决策（纯计算，不依赖 CARLA）。

分割网络把点云逐点分成四类后，控制只需要回答两个问题：

1. **前面这条走廊能不能走？** 把车身宽度方向的走廊投影出来，看走廊内最远的
   连续可行驶区间到哪里，被车辆/行人/障碍物挡住就算堵。
2. **给定轨迹怎么跟？** 用纯跟踪（pure pursuit）算前轮转角，速度由走廊的
   剩余长度决定：越近越慢，进入停车距离就刹停。

所有函数只吃 NumPy 数组，不读配置、不连 CARLA，因此可以离线写测试。
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

import numpy as np

from .labels import DRIVABLE, OBSTACLE, PEDESTRIAN, VEHICLE

# 会挡住车的类别：车辆、行人、障碍物
BLOCKING_CLASSES = (VEHICLE, PEDESTRIAN, OBSTACLE)


def corridor_mask(points: np.ndarray, half_width: float = 1.2,
                  x_min: float = 1.0, x_max: float = 50.0) -> np.ndarray:
    """车身走廊：``x`` 在 [x_min, x_max]、``|y| <= half_width`` 的点。"""
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] < 2:
        raise ValueError('点云至少要有 x、y 两列')
    if half_width <= 0.0:
        raise ValueError('走廊半宽必须为正')
    if x_max <= x_min:
        raise ValueError('走廊长度区间非法：%.2f ~ %.2f' % (x_min, x_max))
    x, y = pts[:, 0], pts[:, 1]
    return (x >= x_min) & (x <= x_max) & (np.abs(y) <= half_width)


def free_distance(points: np.ndarray, classes: np.ndarray, half_width: float = 1.2,
                  x_min: float = 1.0, x_max: float = 50.0) -> float:
    """走廊内最近的阻挡物距离；没有阻挡物时返回 ``x_max``。

    注意只统计"挡住路"的三类，可行驶面的点（路面本身）不算障碍。
    """
    pts = np.asarray(points, dtype=float)
    cls = np.asarray(classes, dtype=int)
    if pts.shape[0] != cls.size:
        raise ValueError('点与类别数量不一致：%d vs %d' % (pts.shape[0], cls.size))
    if pts.shape[0] == 0:
        return float(x_max)

    mask = corridor_mask(pts, half_width, x_min, x_max)
    blocking = np.isin(cls, BLOCKING_CLASSES)
    hit = mask & blocking
    if not hit.any():
        return float(x_max)
    return float(pts[hit, 0].min())


def speed_factor(distance: float, comfort: float = 12.0, stop: float = 4.0) -> float:
    """按剩余距离给出速度系数（0~1），线性过渡。

    * ``distance >= comfort``  -> 1.0（全速）
    * ``distance <= stop``     -> 0.0（刹停）
    * 中间线性插值

    用线性而不是阶跃，是为了避免油门/刹车来回抖。
    """
    if comfort <= stop:
        raise ValueError('comfort 必须大于 stop：%.2f <= %.2f' % (comfort, stop))
    if distance <= stop:
        return 0.0
    if distance >= comfort:
        return 1.0
    return float((distance - stop) / (comfort - stop))


def pure_pursuit(pose: Tuple[float, float, float], path: Sequence[Tuple[float, float]],
                 wheelbase: float = 2.9, lookahead: float = 6.0,
                 max_steer: float = 0.6) -> float:
    """纯跟踪：返回前轮转角（弧度）。

    ``pose`` 是 ``(x, y, yaw)``，``path`` 是车体系下的参考点序列。
    取距离车身约 ``lookahead`` 的那个点，曲率 ``kappa = 2 sin(alpha) / ld``，
    前轮转角 ``delta = atan(wheelbase * kappa)``。
    """
    if wheelbase <= 0.0 or lookahead <= 0.0:
        raise ValueError('轴距与预瞄距离必须为正')
    pts = np.asarray(path, dtype=float)
    if pts.ndim != 2 or pts.shape[1] < 2 or pts.shape[0] == 0:
        raise ValueError('参考轨迹应为非空的 (N, 2) 数组')

    x, y, yaw = pose
    dx = pts[:, 0] - x
    dy = pts[:, 1] - y
    # 转到车体系
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    local_x = dx * cos_yaw + dy * sin_yaw
    local_y = -dx * sin_yaw + dy * cos_yaw

    ahead = local_x > 0.5
    if not ahead.any():
        return 0.0
    dist = np.hypot(local_x[ahead], local_y[ahead])
    order = np.argsort(dist)
    index = order[0]
    for candidate in order:
        if dist[candidate] >= lookahead:
            index = candidate
            break
    target_x = local_x[ahead][index]
    target_y = local_y[ahead][index]
    ld = float(np.hypot(target_x, target_y))
    if ld < 1e-6:
        return 0.0
    alpha = math.atan2(target_y, target_x)
    kappa = 2.0 * math.sin(alpha) / ld
    delta = math.atan(wheelbase * kappa)
    return float(np.clip(delta, -max_steer, max_steer))


def normalized_steer(delta: float, max_steer: float = 0.6) -> float:
    """把前轮转角（弧度）换成 CARLA 的归一化转向指令 ``[-1, 1]``。"""
    if max_steer <= 0.0:
        raise ValueError('最大转角必须为正')
    return float(np.clip(delta / max_steer, -1.0, 1.0))


def straight_path(length: float = 200.0, step: float = 5.0,
                  x0: float = 0.0, y0: float = 0.0):
    """生成一条沿 x 轴的参考轨迹，用于"给定轨迹的控制"演示。"""
    if length <= 0.0 or step <= 0.0:
        raise ValueError('轨迹长度与步长必须为正')
    n = int(length / step) + 1
    return [(x0 + i * step, y0) for i in range(n)]


def drivable_ratio(classes: np.ndarray, half_width: float = 4.0,
                   x_max: float = 30.0, points: np.ndarray | None = None) -> float:
    """前方的可行驶比例，用作"感知是否正常"的在线自检。"""
    cls = np.asarray(classes, dtype=int)
    if cls.size == 0:
        return 0.0
    if points is not None:
        pts = np.asarray(points, dtype=float)
        mask = (pts[:, 0] > 1.0) & (pts[:, 0] <= x_max) & (np.abs(pts[:, 1]) <= half_width)
        if not mask.any():
            return 0.0
        return float(np.mean(cls[mask] == DRIVABLE))
    return float(np.mean(cls == DRIVABLE))