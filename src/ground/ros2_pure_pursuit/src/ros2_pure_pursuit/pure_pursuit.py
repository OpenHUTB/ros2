"""纯追踪（Pure Pursuit）横向控制器。

核心公式：
    基础前视距离  Ld_speed = clip(k * |v| + Ld0, Ld_min, Ld_max)
    曲率自适应    Ld = clip(Ld_speed / (1 + curve_gain * |kappa_path|), Ld_min, Ld_max)
    车体坐标下前视点 (x_g, y_g)
    航向偏差  alpha = atan2(y_g, x_g)
    路径曲率  kappa = 2 * sin(alpha) / Ld
    角速度    omega = kappa * v = 2 * v * sin(alpha) / Ld

前视距离有两层自适应：

1. **随速度**线性增长（原行为）：低速提升跟踪精度，高速抑制振荡；
2. **随路径曲率**缩短（本版新增）：弯道上前视点会自动拉近，避免"抄近路"切内弯
   造成的大横向误差——这正是 S 形路径曲率反号处误差尖峰的来源。
   ``curve_gain=0`` 可退回纯速度自适应，用于对照实验。
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
    curvature: float          # 期望路径曲率（由 alpha 反解）
    target: tuple[float, float]
    cte: float                # 横向跟踪误差
    finished: bool
    path_curvature: float = 0.0   # 前视点附近参考路径的曲率（1/m）


class PurePursuitController:
    def __init__(
        self,
        ld0: float = 0.6,
        k: float = 0.45,
        ld_min: float = 0.4,
        ld_max: float = 2.5,
        goal_tol: float = 0.25,
        curve_gain: float = 2.5,
        curve_span: float = 0.8,
    ) -> None:
        self.ld0 = ld0
        self.k = k
        self.ld_min = ld_min
        self.ld_max = ld_max
        self.goal_tol = goal_tol
        self.curve_gain = curve_gain
        self.curve_span = curve_span

    def lookahead_distance(self, v: float, curvature: float = 0.0) -> float:
        """速度自适应 + 曲率自适应前视距离。

        ``curvature`` 为 0 时退化为原来的 ``clip(k*|v| + ld0, ...)``，
        因此旧调用与旧测试不受影响。
        """
        ld = float(np.clip(self.k * abs(v) + self.ld0, self.ld_min, self.ld_max))
        if self.curve_gain > 0.0 and abs(curvature) > 1e-6:
            ld = ld / (1.0 + self.curve_gain * abs(curvature) * self.curve_span)
            ld = max(ld, self.ld_min)
        return float(ld)

    def compute(self, path: Path, pose: Pose, v: float) -> tuple[float, float, ControlInfo]:
        """给定路径、当前位姿与期望速度，输出 (v_cmd, w_cmd, info)。"""
        idx = path.nearest_index(pose.x, pose.y)
        kappa_path = path.curvature(idx, self.curve_span)
        ld = self.lookahead_distance(v, kappa_path)
        target, end_reached = path.lookahead_point(idx, ld)

        # 转到车体坐标系求航向偏差
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        dx, dy = target[0] - pose.x, target[1] - pose.y
        xg = dx * c + dy * s
        yg = -dx * s + dy * c

        alpha = math.atan2(yg, xg)
        dist = math.hypot(xg, yg)
        eff_ld = max(dist, 1e-6)          # 前视点不足时用实际距离，避免除零
        kappa_cmd = 2.0 * math.sin(alpha) / eff_ld
        w = kappa_cmd * v

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
            curvature=kappa_cmd,
            target=(float(target[0]), float(target[1])),
            cte=float(cte),
            finished=finished,
            path_curvature=float(kappa_path),
        )
        return v, w, info
