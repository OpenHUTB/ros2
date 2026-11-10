"""纯追踪（Pure Pursuit）横向控制器。

核心公式：
    前视距离  Ld = clip(k * |v| + Ld0, Ld_min, Ld_max)
    车体坐标下前视点 (x_g, y_g)
    航向偏差  alpha = atan2(y_g, x_g)
    路径曲率  kappa = 2 * sin(alpha) / Ld
    角速度    omega = kappa * v = 2 * v * sin(alpha) / Ld

前视距离随速度线性增长，可在低速时提升跟踪精度、高速时抑制振荡。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .paths import Path
from .robot import Pose


@dataclass
class ControlInfo:
    """一次控制解算的中间量，用于调试与可视化。"""

    lookahead: float          # 实际前视距离
    alpha: float              # 航向偏差 (rad)
    curvature: float          # 期望路径曲率
    target: tuple[float, float]
    cte: float                # 横向跟踪误差
    finished: bool


class PurePursuitController:
    def __init__(
        self,
        ld0: float = 0.6,
        k: float = 0.45,
        ld_min: float = 0.4,
        ld_max: float = 2.5,
        goal_tol: float = 0.25,
    ) -> None:
        self.ld0 = ld0
        self.k = k
        self.ld_min = ld_min
        self.ld_max = ld_max
        self.goal_tol = goal_tol

    def lookahead_distance(self, v: float) -> float:
        """速度自适应前视距离。"""
        return float(np.clip(self.k * abs(v) + self.ld0, self.ld_min, self.ld_max))

    def compute(self, path: Path, pose: Pose, v: float) -> tuple[float, float, ControlInfo]:
        """给定路径、当前位姿与期望速度，输出 (v_cmd, w_cmd, info)。"""
        idx = path.nearest_index(pose.x, pose.y)
        ld = self.lookahead_distance(v)
        target, end_reached = path.lookahead_point(idx, ld)

        # 转到车体坐标系求航向偏差
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        dx, dy = target[0] - pose.x, target[1] - pose.y
        xg = dx * c + dy * s
        yg = -dx * s + dy * c

        alpha = math.atan2(yg, xg)
        dist = math.hypot(xg, yg)
        eff_ld = max(dist, 1e-6)          # 前视点不足时用实际距离，避免除零
        curvature = 2.0 * math.sin(alpha) / eff_ld
        w = curvature * v

        cte = path.cross_track_error(idx, pose.x, pose.y)
        if path.closed:
            # 闭合路径由调用方按圈数判定完成
            finished = False
        else:
            goal_dist = math.hypot(path.points[-1, 0] - pose.x, path.points[-1, 1] - pose.y)
            finished = end_reached or goal_dist < self.goal_tol

        info = ControlInfo(
            lookahead=ld,
            alpha=alpha,
            curvature=curvature,
            target=(float(target[0]), float(target[1])),
            cte=float(cte),
            finished=finished,
        )
        return v, w, info
