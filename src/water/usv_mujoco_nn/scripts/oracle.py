"""A* 全局路径规划 + 纯跟踪（Pure Pursuit）控制。

仅用于离线生成专家示范（训练标签），部署阶段由神经网络直接输出控制量，
因此答辩时"规划器为神经网络"成立，A* 只是数据生成器。
"""

import heapq

import numpy as np


class GridPlanner(object):
    def __init__(self, obstacles, half=25.0, res=0.5, inflate=2.0):
        self.half = float(half)
        self.res = float(res)
        self.n = int(round(2 * self.half / self.res))
        self.blocked = np.zeros((self.n, self.n), dtype=bool)

        for o in obstacles:
            if o["type"] == "circle":
                r = o["r"] + inflate
                lo = np.array([o["x"] - r, o["y"] - r])
                hi = np.array([o["x"] + r, o["y"] + r])
                i0, j0 = self._w2c(lo)
                i1, j1 = self._w2c(hi)
                for i in range(i0, i1 + 1):
                    for j in range(j0, j1 + 1):
                        p = self._c2w(i, j)
                        if np.hypot(p[0] - o["x"], p[1] - o["y"]) <= r:
                            self.blocked[i, j] = True
            else:
                hx, hy = o["hx"] + inflate, o["hy"] + inflate
                i0, j0 = self._w2c((o["x"] - hx, o["y"] - hy))
                i1, j1 = self._w2c((o["x"] + hx, o["y"] + hy))
                self.blocked[i0:i1 + 1, j0:j1 + 1] = True

        # 边界
        edge = int(round(1.5 / self.res))
        self.blocked[:edge, :] = True
        self.blocked[-edge:, :] = True
        self.blocked[:, :edge] = True
        self.blocked[:, -edge:] = True

    def _w2c(self, p):
        i = int(np.floor((p[1] + self.half) / self.res))
        j = int(np.floor((p[0] + self.half) / self.res))
        return np.clip(i, 0, self.n - 1), np.clip(j, 0, self.n - 1)

    def _c2w(self, i, j):
        return (np.array([j, i], dtype=np.float64) + 0.5) * self.res - self.half

    # ------------------------------------------------------------------ A*
    def plan(self, start, goal, max_expand=60000):
        s = self._w2c(start)
        g = self._w2c(goal)
        if self.blocked[g]:
            g = self._nearest_free(g)
        if self.blocked[s]:
            s = self._nearest_free(s)

        open_heap = [(0.0, s)]
        came = {}
        gscore = {s: 0.0}
        moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
        expanded = 0

        while open_heap:
            _, cur = heapq.heappop(open_heap)
            if cur == g:
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                path.reverse()
                return np.array([self._c2w(i, j) for i, j in path])
            expanded += 1
            if expanded > max_expand:
                break
            for di, dj, cost in moves:
                ni, nj = cur[0] + di, cur[1] + dj
                if not (0 <= ni < self.n and 0 <= nj < self.n):
                    continue
                if self.blocked[ni, nj]:
                    continue
                ng = gscore[cur] + cost
                nb = (ni, nj)
                if ng < gscore.get(nb, 1e18):
                    gscore[nb] = ng
                    came[nb] = cur
                    h = np.hypot(ni - g[0], nj - g[1])
                    heapq.heappush(open_heap, (ng + h, nb))
        return None

    def _nearest_free(self, c, radius=12):
        for r in range(1, radius):
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    ni, nj = c[0] + di, c[1] + dj
                    if 0 <= ni < self.n and 0 <= nj < self.n and not self.blocked[ni, nj]:
                        return (ni, nj)
        return c

    # ------------------------------------------------------------------ 前视点
    def lookahead_point(self, pose, path, lookahead=4.5):
        """返回参考路径上距机器人最近、且沿路径前进 lookahead 米的那一点。"""
        if path is None or len(path) == 0:
            return np.array([pose[0], pose[1]])
        x, y = pose[0], pose[1]
        d = np.hypot(path[:, 0] - x, path[:, 1] - y)
        k = int(np.argmin(d))
        while k < len(path) - 1 and np.hypot(path[k, 0] - x, path[k, 1] - y) < lookahead:
            k += 1
        return path[k]

    # ------------------------------------------------------------------ 纯跟踪
    def pure_pursuit(self, pose, path, lookahead=4.5, v_max=1.15, k_alpha=0.9):
        """几何纯跟踪 + 近目标减速。

        船体艏摇是一阶惯性环节（时间常数约 2 s），直接套经典纯跟踪的
        w = 2v·sin(α)/L 增益过高，会在目标附近形成极限环，故此处降为
        P 型航向控制，并在接近目标时压低线速度。
        """
        if path is None or len(path) == 0:
            return 0.0, 0.0
        x, y, yaw = pose
        d = np.hypot(path[:, 0] - x, path[:, 1] - y)
        idx = int(np.argmin(d))
        k = idx
        while k < len(path) - 1 and np.hypot(path[k, 0] - x, path[k, 1] - y) < lookahead:
            k += 1
        tx, ty = path[k]
        alpha = np.arctan2(ty - y, tx - x) - yaw
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi

        w = float(np.clip(k_alpha * alpha, -1.0, 1.0))

        d_goal = float(np.linalg.norm(path[-1] - np.array([x, y])))
        v_scale = max(0.0, 1.0 - abs(alpha) / 1.1)
        v = v_max * v_scale * min(1.0, 0.25 + d_goal / 3.0)
        return float(v), w
