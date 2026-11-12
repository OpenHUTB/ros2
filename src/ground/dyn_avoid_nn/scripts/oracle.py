"""移动障碍避让：静态 A* 路径 + 纯跟踪 + 动态障碍预测规避。

专家可以看到移动障碍的位置与速度（解析已知），据此预测 T_h 秒后的位置，
如果它落在车头前方、且横向距离小于安全阈值，就叠加一个向空侧的转向偏置并降速。
神经网络只能看到激光雷达，需要自己从帧间差异里推断运动。
"""

import heapq

import numpy as np

from consts import HALF_X, HALF_Y, GOAL, START

RES = 0.5
NX = int(2 * HALF_X / RES)
NY = int(2 * HALF_Y / RES)
MARGIN = 0.75


def is_free(x, y, obstacles, margin=MARGIN):
    if abs(x) > HALF_X - 0.45 or abs(y) > HALF_Y - 0.45:
        return False
    for o in obstacles:
        if abs(x - o["x"]) < o["hx"] + margin and abs(y - o["y"]) < o["hy"] + margin:
            return False
    return True


def _w2c(p):
    return (int(np.clip((p[1] + HALF_Y) / RES, 0, NY - 1)),
            int(np.clip((p[0] + HALF_X) / RES, 0, NX - 1)))


def _c2w(c):
    return (-HALF_X + (c[1] + 0.5) * RES, -HALF_Y + (c[0] + 0.5) * RES)


def plan_static(start, goal, obstacles):
    s, g = _w2c(start), _w2c(goal)
    open_heap = [(0.0, s)]
    came, gs = {}, {s: 0.0}
    moves = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
             (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    seen = 0
    while open_heap and seen < 40000:
        _, cur = heapq.heappop(open_heap)
        if cur == g:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()
            return [_c2w(c) for c in path]
        seen += 1
        for di, dj, cost in moves:
            nb = (cur[0] + di, cur[1] + dj)
            if not (0 <= nb[0] < NY and 0 <= nb[1] < NX):
                continue
            wx, wy = _c2w(nb)
            if not is_free(wx, wy, obstacles):
                continue
            ng = gs[cur] + cost
            if ng < gs.get(nb, 1e18):
                gs[nb] = ng
                came[nb] = cur
                heapq.heappush(open_heap, (ng + np.hypot(nb[0] - g[0], nb[1] - g[1]), nb))
    return [start, goal]


def shortcut(pts, obstacles):
    if len(pts) <= 2:
        return list(pts)
    out, i = [pts[0]], 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not _line_free(pts[i], pts[j], obstacles):
            j -= 1
        out.append(pts[j])
        i = j
    return out


def _line_free(a, b, obstacles, step=0.2):
    n = max(int(np.hypot(b[0] - a[0], b[1] - a[1]) / step), 1)
    for t in np.linspace(0.0, 1.0, n + 1):
        if not is_free(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, obstacles):
            return False
    return True


def build_path(obstacles, start, goal):
    raw = plan_static(start, goal, obstacles)
    pts = shortcut(raw, obstacles)
    dense = [pts[0]]
    for i in range(1, len(pts)):
        a, b = pts[i - 1], pts[i]
        n = max(int(np.hypot(b[0] - a[0], b[1] - a[1]) / 0.2), 1)
        for t in np.linspace(0.0, 1.0, n + 1)[1:]:
            dense.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return np.asarray(dense, dtype=np.float64)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class DynExpert(object):
    """纯跟踪 + 移动障碍预测规避。"""

    def __init__(self, path, v_max=0.85, lookahead=1.2, k_alpha=1.3,
                 horizon=1.4, safe_lat=1.9, mover_r=0.55):
        self.path = path
        self.v_max = v_max
        self.lookahead = lookahead
        self.k_alpha = k_alpha
        self.horizon = horizon
        self.safe_lat = safe_lat
        self.mover_r = mover_r
        self.k = 0

    def _safe_speed(self, k, v_cmd, movers, x, y, step=0.2):
        """前向仿真：沿路径按候选速度推进，检查与移动障碍的最小距离。
        返回可用速度；若所有速度都冲突且停车也不安全，返回 -1。"""
        n = len(self.path)
        for v in (v_cmd, 0.6 * v_cmd, 0.3 * v_cmd, 0.0):
            ok = True
            for t in np.arange(0.1, 2.01, 0.1):
                idx = min(k + int(v * t / step), n - 1)
                px, py = self.path[idx]
                for mx, my, mvx, mvy in movers:
                    if np.hypot(mx + mvx * t - px, my + mvy * t - py) < 1.35:
                        ok = False
                        break
                if not ok:
                    break
            if ok:
                return v
        # 停车也不安全
        for t in np.arange(0.1, 2.01, 0.1):
            for mx, my, mvx, mvy in movers:
                if np.hypot(mx + mvx * t - x, my + mvy * t - y) < 1.35:
                    return -1.0
        return 0.0

    def act(self, pose, speed=0.0, yaw_rate=0.0, movers=None, scan=None):
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

        w = self.k_alpha * alpha - 0.55 * yaw_rate
        v = self.v_max * max(0.0, 1.0 - abs(alpha) / 1.4)

        # 移动障碍规避：这些障碍是沿 y 扫动的，横向让开没有意义，
        # 关键是"过线时机"。对每档候选速度前向仿真 2 s，选不会冲突的最大速度。
        if movers:
            v = self._safe_speed(k, v, movers, x, y)
            if v < 0:                      # 连停车都不安全 -> 后退让开
                c, s = np.cos(yaw), np.sin(yaw)
                lat = 0.0
                for mx, my, _, _ in movers:
                    lat += -1.0 * ((mx - x) * s - (my - y) * c)
                side = -1.0 if lat > 0 else 1.0
                return -0.28, float(np.clip(side * 0.5, -0.8, 0.8))
        w = float(np.clip(w, -1.2, 1.2))

        if scan is not None:
            m = len(scan)
            half = m // 4
            front = scan[m // 2 - half // 2: m // 2 + half // 2]
            d_front = float(np.min(front)) if len(front) else 9.9
            if d_front < 0.70:
                return -0.15, 0.6 if float(np.min(scan[:m // 2])) > float(np.min(scan[m // 2:])) else -0.6
            if d_front < 1.05:
                v = min(v, 0.15)

        if k >= n - 1 and np.hypot(tx - x, ty - y) < 0.4:
            return 0.0, 0.0
        return max(v, 0.10), w
