"""狭窄通道通行：开口选择 + 纯跟踪控制器。

专家可以看到开口的真实几何（宽度与中心），据此挑一个"过得去且绕路少"的开口，
再沿 起点 → 开口中心 → 目标 的折线做纯跟踪。
神经网络只有激光雷达，需要自己从扫描里认出哪些开口能过。
"""

import numpy as np

from sim_core import GAPS, MIN_PASSABLE, ROBOT_HALF_W


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def passable_gaps():
    out = []
    for g in GAPS:
        w = g["hi"] - g["lo"]
        if w >= MIN_PASSABLE:
            out.append({"lo": g["lo"], "hi": g["hi"], "width": w,
                        "center": 0.5 * (g["lo"] + g["hi"])})
    return out


def choose_gap(start, goal):
    """按"绕路最少"挑选开口。"""
    gaps = passable_gaps()
    if not gaps:
        return None
    sy, gy = start[1], goal[1]
    best, best_cost = None, 1e9
    for g in gaps:
        cost = abs(g["center"] - sy) + abs(g["center"] - gy)
        if cost < best_cost:
            best, best_cost = g, cost
    return best


def build_path(start, goal):
    g = choose_gap(start, goal)
    if g is None:
        pts = [start, goal]
    else:
        pts = [start, (0.0, g["center"]), goal]
    dense = [pts[0]]
    for i in range(1, len(pts)):
        a, b = pts[i - 1], pts[i]
        n = max(int(np.hypot(b[0] - a[0], b[1] - a[1]) / 0.15), 1)
        for t in np.linspace(0.0, 1.0, n + 1)[1:]:
            dense.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return np.asarray(dense, dtype=np.float64), g


class GapExpert(object):
    def __init__(self, path, v_max=0.80, lookahead=1.1, k_alpha=1.4):
        self.path = path
        self.v_max = v_max
        self.lookahead = lookahead
        self.k_alpha = k_alpha
        self.k = 0

    def act(self, pose, speed=0.0, yaw_rate=0.0, scan=None):
        x, y, yaw = pose
        n = len(self.path)
        k = self.k
        d_cur = np.hypot(self.path[k, 0] - x, self.path[k, 1] - y)
        if d_cur <= 2.0 * self.lookahead:
            best, best_d = k, d_cur
            j = k
            while j < n - 1:
                j += 1
                d = np.hypot(self.path[j, 0] - x, self.path[j, 1] - y)
                if d < best_d:
                    best, best_d = j, d
                elif d > best_d + 2.0:
                    break
            k = best
        while k < n - 1 and np.hypot(self.path[k, 0] - x, self.path[k, 1] - y) < self.lookahead:
            k += 1
        self.k = k
        tx, ty = self.path[k]
        alpha = wrap(np.arctan2(ty - y, tx - x) - yaw)

        w = float(np.clip(self.k_alpha * alpha - 0.55 * yaw_rate, -1.1, 1.1))
        v = self.v_max * max(0.0, 1.0 - abs(alpha) / 1.4)

        if scan is not None:
            m = len(scan)
            front = scan[int(0.30 * m):int(0.70 * m)]
            d_front = float(np.min(front)) if len(front) else 9.9
            if d_front < 0.60:
                left = float(np.min(scan[int(0.70 * m):]))
                right = float(np.min(scan[:int(0.30 * m)]))
                return -0.15, (0.55 if left > right else -0.55)
            if d_front < 0.95:
                v = min(v, 0.15)

        if k >= n - 1 and np.hypot(tx - x, ty - y) < 0.35:
            return 0.0, 0.0
        return max(v, 0.10), w
