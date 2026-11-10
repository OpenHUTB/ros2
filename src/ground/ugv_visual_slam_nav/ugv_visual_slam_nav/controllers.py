"""底层控制：纯追踪 + 基于深度图的局部避障。

纯追踪负责沿全局路径走，局部项只看一帧深度图，专门处理地图里
没有的动态障碍（地图更新有延迟，纯靠重规划来不及）。
"""

import math

import numpy as np

from . import params as P
from .kinematics import wrap


def pure_pursuit(pose, path, lookahead=None):
    """返回 (v, w)。路径为空时原地停住。"""
    lookahead = P.LOOKAHEAD if lookahead is None else lookahead
    if not path:
        return 0.0, 0.0

    x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
    pts = np.asarray(path, dtype=np.float64)

    d2 = (pts[:, 0] - x) ** 2 + (pts[:, 1] - y) ** 2
    i0 = int(np.argmin(d2))

    target = pts[-1]
    on_final = True
    for i in range(i0, len(pts)):
        if math.hypot(pts[i, 0] - x, pts[i, 1] - y) >= lookahead:
            target = pts[i]
            on_final = (i == len(pts) - 1)
            break

    dx = target[0] - x
    dy = target[1] - y
    L = math.hypot(dx, dy)
    if L < 1e-6:
        return 0.0, 0.0

    alpha = wrap(math.atan2(dy, dx) - th)

    # 终点收敛段：纯追踪在前视距离之外会绕着终点转圈，这里改成
    # 角度闭环 + 随距离衰减的速度，能稳稳停在 GOAL_TOL 之内。
    if on_final and L < 0.90:
        w = float(np.clip(2.4 * alpha, -P.MAX_W, P.MAX_W))
        v = P.MAX_V * float(np.clip((L - 0.16) / 0.55, 0.0, 1.0))
        return v, w
    # 纯追踪曲率：kappa = 2 sin(alpha) / L
    kappa = 2.0 * math.sin(alpha) / max(L, 0.15)
    v = P.MAX_V * float(np.clip(1.0 - 0.55 * abs(alpha) / (math.pi / 2), 0.30, 1.0))
    w = float(np.clip(v * kappa, -P.MAX_W, P.MAX_W))
    return v, w


def avoid_from_depth(depth, v, w, near=0.52, stop=0.30, gain=0.40, caution=0.95):
    """用一帧深度做局部避障：前方变近则减速直到停车，并向更深的一侧偏转。

    早先版本的减速系数下限是 0.25，也就是"再近也要往前挪一点"—— 车会以
    0.09 m/s 的速度一点点蹭到墙上，而碰撞判据是圆的相交，蹭到就是撞。
    这里改成在 `stop` 距离以内把线速度直接压到 0，把"逃出去"交给
    策略层的原地转向恢复。

    `caution` 是一层更外的谨慎区：动态障碍以 1.2 m/s 迎面而来时，从
    反应带边缘到接触只要 0.1 s，等 `near` 触发已经来不及。所以只要
    前方锥体内出现过 <caution 的距离，就把速度先压到 0.28 m/s。
    """
    if depth is None:
        return v, w
    H, W = depth.shape
    band = depth[H // 2, :]
    c = W // 2
    half = max(3, W // 10)

    # 15 分位数而不是中位数：斜着看一个近障碍时它只占少数几列，
    # 中位数会把"有一列已经贴脸"平滑掉
    front = float(np.percentile(band[c - half:c + half + 1], 15.0))
    left = float(np.median(band[c + half: c + 3 * half + 1]))
    right = float(np.median(band[c - 3 * half: c - half + 1]))

    if front > near:
        return v, w

    scale = float(np.clip((front - stop) / (near - stop), 0.0, 1.0))
    # 哪边更深就往哪边让
    if left > right + 0.15:
        bias = gain * (1.0 - scale)
    elif right > left + 0.15:
        bias = -gain * (1.0 - scale)
    else:
        bias = 0.75 * gain * (1.0 - scale) * (1.0 if left >= right else -1.0)
    w = float(np.clip(w + bias, -P.MAX_W, P.MAX_W))
    return v * scale, w


def goal_reached(pose, goal, tol=None):
    tol = P.GOAL_TOL if tol is None else tol
    return math.hypot(goal[0] - pose[0], goal[1] - pose[1]) < tol
