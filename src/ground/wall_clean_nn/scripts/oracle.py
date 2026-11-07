"""沿墙清扫：路径生成与纯跟踪控制。

任务：清扫车从房间内部的任意位置出发，先驶向靠墙走廊，然后绕房间一周，
把距墙 1 m 以内的区域扫干净。

路径 = 圆角矩形闭合轮廓（离墙 0.85 m）+ 一段从起点到轮廓的直线过渡。
轮廓上没有 U 型掉头，转弯半径为 0.7 m，正是差速车能做到的量级。
"""

import numpy as np

from sim_core import HALF_X, HALF_Y

WALL_IN_X = HALF_X - 0.30      # 墙内表面 x
WALL_IN_Y = HALF_Y - 0.30
OFFSET = 0.85                  # 离墙距离
CORNER_R = 0.7


def is_free(x, y, obstacles, margin):
    if abs(x) > HALF_X - 0.85 or abs(y) > HALF_Y - 0.85:
        return False
    for o in obstacles:
        if abs(x - o["x"]) < o["hx"] + margin and abs(y - o["y"]) < o["hy"] + margin:
            return False
    return True


def sample_start(rng, obstacles, margin=0.9, tries=200):
    """在房间内部随机采样一个不与家具/墙重叠的起点。"""
    for _ in range(tries):
        x = float(rng.uniform(-4.6, 4.6))
        y = float(rng.uniform(-3.6, 3.6))
        if is_free(x, y, obstacles, margin):
            return (x, y, float(rng.uniform(-np.pi, np.pi)))
    return (-4.6, 3.6, 0.7)


def perimeter_path(step=0.15):
    """圆角矩形闭合轮廓，按逆时针采样。"""
    ax = WALL_IN_X - OFFSET - CORNER_R
    ay = WALL_IN_Y - OFFSET - CORNER_R
    corners = [(ax, ay, 0.0), (-ax, ay, np.pi / 2), (-ax, -ay, np.pi), (ax, -ay, 1.5 * np.pi)]
    pts = []
    for idx, (cx, cy, a0) in enumerate(corners):
        n = max(int(CORNER_R * np.pi / 2 / step), 6)
        for t in np.linspace(0.0, np.pi / 2, n + 1):
            a = a0 + t
            pts.append((cx + CORNER_R * np.cos(a), cy + CORNER_R * np.sin(a)))
        # 直边：从当前角末端到下一个角起点
        nxt = corners[(idx + 1) % 4]
        end = (nxt[0] + CORNER_R * np.cos(nxt[2]), nxt[1] + CORNER_R * np.sin(nxt[2]))
        cur = pts[-1]
        seg = float(np.hypot(end[0] - cur[0], end[1] - cur[1]))
        n = max(int(seg / step), 1)
        for t in np.linspace(0.0, 1.0, n + 1)[1:]:
            pts.append((cur[0] + (end[0] - cur[0]) * t, cur[1] + (end[1] - cur[1]) * t))
    return np.asarray(pts, dtype=np.float64)


def _nearest_index(path, p):
    d = np.hypot(path[:, 0] - p[0], path[:, 1] - p[1])
    return int(np.argmin(d))


def build_wall_path(obstacles, start):
    """起点 -> 轮廓最近点 -> 绕行一周回到该点。"""
    loop = perimeter_path()
    i0 = _nearest_index(loop, start)
    order = np.concatenate([np.arange(i0, len(loop)), np.arange(0, i0 + 1)])
    lap = loop[order]
    sx, sy = float(start[0]), float(start[1])
    head = np.array([[sx, sy], [loop[i0, 0], loop[i0, 1]]], dtype=np.float64)
    return np.vstack([head, lap[1:]]), lap


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class WallExpert(object):
    """沿墙纯跟踪：前视 + 角速度阻尼 + 雷达安全层。"""

    def __init__(self, path, v_max=0.65, lookahead=1.0, k_alpha=1.3):
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
        # 目标已足够近时，吸附到前方最近点；否则保持索引，避免路径折返处跳段
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

        w = float(np.clip(self.k_alpha * alpha - 0.55 * yaw_rate, -1.0, 1.0))
        v = self.v_max * max(0.0, 1.0 - abs(alpha) / 1.4)

        if scan is not None:
            m = len(scan)
            front = scan[int(0.25 * m):int(0.75 * m)]
            d_front = float(np.min(front)) if len(front) else 9.9
            if d_front < 0.80:
                left_min = float(np.min(scan[int(0.70 * m):]))
                right_min = float(np.min(scan[:int(0.30 * m)]))
                return -0.15, (0.55 if left_min > right_min else -0.55)
            if d_front < 1.15:
                v = min(v, 0.12)

        if k >= n - 1 and np.hypot(tx - x, ty - y) < 0.35:
            return 0.0, 0.0
        return max(v, 0.10), w
