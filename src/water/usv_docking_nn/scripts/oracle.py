"""靠泊专家控制器：几何纯跟踪（与 USV 避障项目同一套控制律）。

路径在每个回合按起点动态生成：起点 -> 入口引导点 (10, 0) -> 沿泊位轴线到 17.2。
纯跟踪对一阶惯性艏摇是稳定的，P 型航向环会振荡（实测过）。
"""

import numpy as np

from sim_core import BERTH_X


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def build_path(start, guide_x=8.0, dock_x=17.0, step=0.5):
    pts = [np.array([start[0], start[1]], dtype=np.float64),
           np.array([guide_x, 0.0])]
    x = guide_x
    while x < dock_x:
        x += step
        pts.append(np.array([min(x, dock_x), 0.0]))
    return np.asarray(pts)


class DockExpert(object):
    def __init__(self, path, v_max=0.85, lookahead=3.0, k_alpha=0.6, stop_dist=0.40):
        self.path = path
        self.v_max = v_max
        self.lookahead = lookahead
        self.k_alpha = k_alpha
        self.stop_dist = stop_dist
        self.k = 0

    def act(self, pose, speed=0.0, yaw_rate=0.0):
        x, y, yaw = pose
        # 单调推进的寻点索引：只向前看，避免越过路径起点后目标跳回身后
        k = self.k
        while k < len(self.path) - 1 and np.hypot(self.path[k, 0] - x, self.path[k, 1] - y) < self.lookahead:
            k += 1
        self.k = k
        tx, ty = self.path[k]
        alpha = wrap(np.arctan2(ty - y, tx - x) - yaw)

        d_end = float(np.hypot(self.path[-1, 0] - x, self.path[-1, 1] - y))

        # 末段：低速对准泊位轴线，避免纯跟踪在终点附近追身后点而打转
        if d_end <= 9.0:
            yaw_d = float(np.clip(-0.7 * y, -0.5, 0.5))
            err = wrap(yaw_d - yaw)
            w = float(np.clip(0.8 * err - 0.30 * yaw_rate, -0.5, 0.5))
            v = 0.40 if d_end > 1.2 else max(0.10, 0.40 * d_end)
            if abs(y) < 0.30 and abs(yaw) < 0.12 and d_end < self.stop_dist:
                v, w = 0.0, 0.0
                return v, w
            # 已贴近岸墙但横向仍偏：先退出再重新对准
            if x > 16.6 and abs(y) > 0.55:
                return -0.22, float(np.clip(1.0 * err - 0.7 * yaw_rate, -0.55, 0.55))
            return v, w

        w = float(np.clip(self.k_alpha * alpha - 0.30 * yaw_rate, -0.6, 0.6))
        v = self.v_max * max(0.0, 1.0 - abs(alpha) / 1.2)
        v = min(v, 0.35 + 0.55 * d_end)
        return v, w
