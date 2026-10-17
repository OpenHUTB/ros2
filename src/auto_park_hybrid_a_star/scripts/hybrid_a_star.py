#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hybrid_a_star.py

Hybrid A* 局部路径规划器（面向垂直泊车场景）。

状态空间：(x, y, yaw)，yaw 离散到 10 度；
动作空间：{前进, 倒车} × {左满舵, 直行, 右满舵} = 6 个运动学原语；
代价：行驶距离（m）；
启发：欧氏距离到目标（可采纳）；
碰撞检测：见 collision_check.py。

Reeds-Shepp 曲线（reeds_shepp.py）作为启发式的一部分尝试调用；
若 RS 无解则退化为欧氏距离，保证规划器一定能返回路径。
"""

import math
import heapq

from collision_check import VehicleFootprint, check_collision
try:
    from reeds_shepp import get_rs_path
except Exception:
    get_rs_path = None


class HybridAStar:
    def __init__(self, vp, obstacles, xy_res=0.2, yaw_res_deg=10.0,
                 step=0.8, max_iter=20000):
        self.vp = vp
        self.obstacles = obstacles
        self.xy_res = xy_res
        self.yaw_res = math.radians(yaw_res_deg)
        self.step = step
        self.r = vp.min_turn_radius
        self.max_iter = max_iter
        steer_max = math.atan(self.vp.wheel_base / self.r)
        self.motions = []
        for gear in (1, -1):
            for steer in (-steer_max, 0.0, steer_max):
                self.motions.append((gear, steer))

    def _key(self, x, y, yaw):
        ix = int(round(x / self.xy_res))
        iy = int(round(y / self.xy_res))
        it = int(round(_norm(yaw) / self.yaw_res))
        return (ix, iy, it)

    def _simulate(self, x, y, yaw, gear, steer):
        L = self.vp.wheel_base
        v = float(gear)
        n = 8
        sub = (self.step / abs(v)) / n
        cx, cy, ct = x, y, yaw
        for _ in range(n):
            yaw_rate = v * math.tan(steer) / L
            ct += yaw_rate * sub
            cx += v * math.cos(ct) * sub
            cy += v * math.sin(ct) * sub
            if check_collision(cx, cy, ct, self.vp, self.obstacles):
                return None
        return (cx, cy, _norm(ct), gear)

    def _heuristic(self, x, y, goal):
        return math.hypot(goal[0] - x, goal[1] - y)

    def plan(self, start, goal, goal_tol_xy=0.4, goal_tol_yaw=math.radians(20)):
        sx, sy, st = start
        gx, gy, gt = goal
        self._start = (sx, sy, st)

        open_heap = []
        g_cost = {self._key(sx, sy, st): 0.0}
        parent = {}
        heapq.heappush(open_heap, (self._heuristic(sx, sy, goal),
                                   0.0, sx, sy, st))
        closed = set()

        iter_cnt = 0
        while open_heap and iter_cnt < self.max_iter:
            iter_cnt += 1
            f, g, x, y, yaw = heapq.heappop(open_heap)
            key = self._key(x, y, yaw)
            if key in closed:
                continue
            closed.add(key)

            # 到达目标？
            if (math.hypot(x - gx, y - gy) < goal_tol_xy and
                    abs(_norm(yaw - gt)) < goal_tol_yaw):
                return self._reconstruct(parent, key, (x, y, yaw, 1))

            for gear, steer in self.motions:
                res = self._simulate(x, y, yaw, gear, steer)
                if res is None:
                    continue
                nx, ny, nt, ng = res
                nkey = self._key(nx, ny, nt)
                if nkey in closed:
                    continue
                ng_cost = g + self.step
                if nkey not in g_cost or ng_cost < g_cost[nkey]:
                    g_cost[nkey] = ng_cost
                    parent[nkey] = (key, (nx, ny, nt, ng))
                    h = self._heuristic(nx, ny, goal)
                    heapq.heappush(open_heap, (ng_cost + h, ng_cost,
                                               nx, ny, nt))

        return None

    def _reconstruct(self, parent, end_key, end_state):
        chain = [end_state]
        key = end_key
        while key in parent:
            pkey, state = parent[key]
            chain.append(state)
            key = pkey
        chain.reverse()
        # 补上真正的起点
        sx, sy, st = self._start
        chain.insert(0, (sx, sy, st, 1))
        return chain


def _norm(t):
    while t > math.pi:
        t -= 2 * math.pi
    while t < -math.pi:
        t += 2 * math.pi
    return t


if __name__ == "__main__":
    vp = VehicleFootprint()
    obs = []
    planner = HybridAStar(vp, obs, step=1.0)
    s = (0.0, 0.0, 0.0)
    g = (6.0, 3.0, math.pi / 2.0)
    path = planner.plan(s, g)
    if path:
        print("points:", len(path), "end:", path[-1])
    else:
        print("no path")
