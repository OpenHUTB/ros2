"""RRT（快速扩展随机树）二维路径规划，纯标准库实现。

算法（LaValle, 1998）
--------------------
1. 树从起点 q_start 开始，只含一个节点。
2. 循环：在自由空间随机采样 q_rand；找到树上离它最近的节点 q_near；
   从 q_near 朝 q_rand 方向前进一个步长 step 得到 q_new；
   若线段 q_near→q_new 无碰撞，把 q_new 加入树，并记录父节点。
3. 当新节点落在目标容差半径内，回溯父指针得到路径。
4. 后处理：随机捷径（shortcut）平滑。反复随机取路径上两点，若直连无碰撞
   就用直连替换中间那一段，从而去掉锯齿。

纯标准库关键点：只有线段-矩形相交检测和最近邻搜索，无需 KD 树（节点数少时
线性扫描足够，O(n) 每次扩展）。
"""

import math

# ---------------------------------------------------------------------------
# 伪随机
# ---------------------------------------------------------------------------


class Rng:
    def __init__(self, seed=1):
        self.state = seed & 0x7FFFFFFF

    def uniform(self, lo=0.0, hi=1.0):
        self.state = (1103515245 * self.state + 12345) & 0x7FFFFFFF
        u = (self.state + 0.5) / 2147483648.0
        return lo + (hi - lo) * u

    def randint(self, n):
        return int(self.uniform(0, n)) % n if n > 0 else 0


# ---------------------------------------------------------------------------
# 几何
# ---------------------------------------------------------------------------


def point_in_rect(p, rect, margin=0.0):
    x, y = p
    x0, y0, x1, y1 = rect
    return (x0 - margin) <= x <= (x1 + margin) and (y0 - margin) <= y <= (y1 + margin)


def segment_hits_rect(p, q, rect, margin=0.0):
    """线段 pq 与（外扩 margin 的）轴对齐矩形是否相交。slab 裁剪法。"""
    x0, y0, x1, y1 = (rect[0] - margin, rect[1] - margin,
                      rect[2] + margin, rect[3] + margin)
    (px, py), (qx, qy) = p, q
    dx, dy = qx - px, qy - py
    t0, t1 = 0.0, 1.0
    for d, lo, hi, start in ((dx, x0, x1, px), (dy, y0, y1, py)):
        if abs(d) < 1e-12:
            if start < lo or start > hi:
                return False
            continue
        a = (lo - start) / d
        b = (hi - start) / d
        if a > b:
            a, b = b, a
        t0 = max(t0, a)
        t1 = min(t1, b)
        if t0 > t1:
            return False
    return True


def segment_free(p, q, obstacles, margin=1.5):
    for r in obstacles:
        if segment_hits_rect(p, q, r, margin):
            return False
    return True


def dist(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


# ---------------------------------------------------------------------------
# RRT
# ---------------------------------------------------------------------------


class RRT:
    def __init__(self, start, goal, obstacles, bounds=(0.0, 100.0, 0.0, 100.0),
                 step=6.0, goal_tol=4.0, goal_bias=0.12, max_iters=4000,
                 seed=3, margin=1.5):
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.obstacles = list(obstacles)
        self.bounds = bounds
        self.step = step
        self.goal_tol = goal_tol
        self.goal_bias = goal_bias
        self.max_iters = max_iters
        self.margin = margin
        self.rng = Rng(seed)
        self.nodes = [self.start]
        self.parent = [-1]
        self.edges = []            # 用于绘图的树边

    # -- 采样 --------------------------------------------------------------
    def sample(self):
        if self.rng.uniform() < self.goal_bias:
            return self.goal
        x0, x1, y0, y1 = self.bounds
        return (self.rng.uniform(x0, x1), self.rng.uniform(y0, y1))

    # -- 最近邻 ------------------------------------------------------------
    def nearest(self, q):
        best, bi = None, -1
        for i, n in enumerate(self.nodes):
            d = dist(n, q)
            if best is None or d < best:
                best, bi = d, i
        return bi

    # -- 朝目标前进一个步长 ------------------------------------------------
    def steer(self, q_from, q_to):
        d = dist(q_from, q_to)
        if d <= self.step:
            return q_to
        r = self.step / d
        return (q_from[0] + (q_to[0] - q_from[0]) * r,
                q_from[1] + (q_to[1] - q_from[1]) * r)

    # -- 主循环 ------------------------------------------------------------
    def plan(self):
        for _ in range(self.max_iters):
            q_rand = self.sample()
            i_near = self.nearest(q_rand)
            q_new = self.steer(self.nodes[i_near], q_rand)
            x0, x1, y0, y1 = self.bounds
            if not (x0 <= q_new[0] <= x1 and y0 <= q_new[1] <= y1):
                continue
            if any(point_in_rect(q_new, r, self.margin) for r in self.obstacles):
                continue
            if not segment_free(self.nodes[i_near], q_new, self.obstacles, self.margin):
                continue
            self.nodes.append(q_new)
            self.parent.append(i_near)
            self.edges.append((self.nodes[i_near], q_new))
            if dist(q_new, self.goal) <= self.goal_tol:
                if segment_free(q_new, self.goal, self.obstacles, self.margin):
                    self.nodes.append(self.goal)
                    self.parent.append(len(self.nodes) - 2)
                    self.edges.append((q_new, self.goal))
                    return self._path(len(self.nodes) - 1)
        return None

    def _path(self, idx):
        path = []
        while idx != -1:
            path.append(self.nodes[idx])
            idx = self.parent[idx]
        return path[::-1]


def path_length(path):
    return sum(dist(path[i], path[i + 1]) for i in range(len(path) - 1))


def smooth(path, obstacles, iterations=300, seed=5, margin=1.5):
    """随机捷径平滑：把能直连的曲折段替换成直线。"""
    if not path or len(path) < 3:
        return list(path)
    rng = Rng(seed)
    out = list(path)
    for _ in range(iterations):
        n = len(out)
        if n < 3:
            break
        i = rng.randint(n - 1)
        j = rng.randint(n - 1)
        if i == j or abs(i - j) < 2:
            continue
        a, b = min(i, j), max(i, j)
        if segment_free(out[a], out[b], obstacles, margin):
            out = out[:a + 1] + out[b:]
    return out


def path_is_free(path, obstacles, margin=1.5):
    return all(segment_free(path[i], path[i + 1], obstacles, margin)
               for i in range(len(path) - 1))


def densify(path, max_step=0.5):
    """把折线按 <= max_step 的弧长等距重采样，端点与总长度保持不变。

    随机捷径平滑会留下很长的直线段（10 个顶点就能跨 200 多米）。纯跟踪若
    把「下一个顶点」当前视目标，会直接朝几十米外的拐角顶点切过去。加密后
    每个路点间隔很小，前视点才落在真正的前方距离处，发布的 Path 也更平滑。
    """
    if len(path) < 2:
        return list(path)
    out = [tuple(path[0])]
    for a, b in zip(path[:-1], path[1:]):
        seg = dist(a, b)
        if seg <= 1e-12:
            continue
        n = int(math.ceil(seg / max_step))
        for k in range(1, n + 1):
            t = k / n
            out.append((a[0] + (b[0] - a[0]) * t,
                        a[1] + (b[1] - a[1]) * t))
    return out


def lookahead_point(path, x, y, lookahead):
    """纯跟踪前视点：把 (x,y) 投影到最近的线段，再沿路径累计弧长
    ``lookahead``，在跨越的那一段上线性插值返回精确目标点。

    旧实现是「累计弧长超过 lookahead 就返回下一个顶点」。平滑后的顶点很稀疏，
    那个顶点可能在几十米外、甚至已经在拐角之后，车辆会朝它直线切角撞进障碍。
    这里的插值版本与路径分辨率无关，稀疏/加密路径给出同一目标点。
    """
    if not path:
        return None
    if len(path) == 1:
        return path[0]

    best_d, best_k, best_p = float("inf"), 0, path[0]
    for k in range(len(path) - 1):
        a, b = path[k], path[k + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        d2 = dx * dx + dy * dy
        if d2 <= 1e-18:
            continue
        t = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / d2))
        p = (a[0] + dx * t, a[1] + dy * t)
        d = dist(p, (x, y))
        if d < best_d:
            best_d, best_k, best_p = d, k, p

    remaining, start = float(lookahead), best_p
    for k in range(best_k, len(path) - 1):
        b = path[k + 1]
        seg = dist(start, b)
        if seg <= 1e-12:
            continue
        if remaining <= seg:
            t = remaining / seg
            return (start[0] + (b[0] - start[0]) * t,
                    start[1] + (b[1] - start[1]) * t)
        remaining -= seg
        start = b
    return path[-1]


# 默认场景
START = (8.0, 8.0)
GOAL = (92.0, 92.0)
OBSTACLES = [
    (25.0, 0.0, 40.0, 55.0),
    (55.0, 30.0, 70.0, 100.0),
    (40.0, 70.0, 60.0, 82.0),
    (78.0, 55.0, 96.0, 66.0),
]
