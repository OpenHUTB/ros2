# -*- coding: utf-8 -*-
"""两种局部规划器：DWA 与 APF。

改写自 AtsushiSakai/PythonRobotics（MIT），统一接口：
    step(pose, scan, goal, dt) -> (v, w)
scan 为二维激光点（世界坐标）。相对上游的改动见 THIRD_PARTY_NOTICES.md。
"""

import math

import numpy as np


def scan_to_points(pose, scan):
    """激光 (angle, dist) → 世界坐标点。"""
    x, y, _ = pose
    pts = []
    for ang, dist in scan:
        pts.append((x + dist * math.cos(ang), y + dist * math.sin(ang)))
    return np.array(pts) if pts else np.zeros((0, 2))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class BasePlanner:
    name = "base"

    def reset(self):
        pass

    def step(self, pose, scan, goal, dt):
        raise NotImplementedError


HORIZON_STEPS = 8          # 动态窗口覆盖多少个预测步（_predict 步长 0.1 s）


class DWAPlanner(BasePlanner):
    """动态窗口法：在可达速度窗口内采样，按 朝向/障碍/速度 三项打分。"""

    name = "dwa"

    def __init__(self, v_max=0.8, w_max=1.5, acc=0.6, w_acc=2.0,
                 alpha=2.0, beta=3.0, gamma=0.4):
        self.v_max, self.w_max = v_max, w_max
        self.acc, self.w_acc = acc, w_acc
        self.alpha, self.beta, self.gamma = alpha, beta, gamma
        self.v, self.w = 0.0, 0.0

    def reset(self):
        self.v, self.w = 0.0, 0.0

    def _predict(self, pose, v, w, horizon=1.2, dt=0.1):
        x, y, yaw = pose
        pts = []
        t = 0.0
        while t < horizon:
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw = wrap(yaw + w * dt)
            pts.append((x, y, yaw))
            t += dt
        return pts

    def step(self, pose, scan, goal, dt):
        pts = scan_to_points(pose, scan)
        # 动态窗口：只采样本周期内由加速度可达的速度。
        # HORIZON_STEPS 取 8，对应 _predict 里 0.8 s 的预测跨度；否则起步 v=0
        # 时窗口几乎闭合会卡死。
        dv = self.acc * dt * HORIZON_STEPS
        dw = self.w_acc * dt * HORIZON_STEPS
        v_lo = max(0.05, self.v - dv)
        v_hi = min(self.v_max, self.v + dv)
        w_lo = max(-self.w_max, self.w - dw)
        w_hi = min(self.w_max, self.w + dw)
        v_lo = min(v_lo, v_hi - 0.05)          # 保证窗口非空

        # 先剔除会撞上障碍的候选（硬约束），再在可行解里按 朝向+速度 择优。
        # 之前把障碍代价直接减进总代价，符号导致"越贴障越优"，会直接撞穿。
        best, best_cost = (0.05, 0.0), -1e9
        fallback, fallback_d = (0.05, 0.0), -1e9
        safe = 0.50
        feasible = 0
        for v in np.linspace(v_lo, v_hi, 6):
            for w in np.linspace(w_lo, w_hi, 9):
                traj = self._predict(pose, v, w)
                end = traj[-1]
                dmin = 1e9
                if len(pts):
                    for (px, py, _) in traj[::2]:
                        dmin = min(dmin, float(np.hypot(pts[:, 0] - px, pts[:, 1] - py).min()))
                if dmin > fallback_d:                 # 记录最安全的候选作兜底
                    fallback_d, fallback = dmin, (float(v), float(w))
                if dmin < safe:
                    continue
                feasible += 1
                head = abs(wrap(math.atan2(goal[1] - end[1], goal[0] - end[0]) - end[2]))
                cost = self.alpha * (math.pi - head) + self.gamma * v
                if cost > best_cost:
                    best_cost, best = cost, (float(v), float(w))
        if feasible == 0:
            best = fallback
        self.v, self.w = best
        return self.v, self.w


class APFPlanner(BasePlanner):
    """人工势场法：目标引力 + 障碍斥力，合成方向转成 (v, w)。"""

    name = "apf"

    def __init__(self, v_max=0.8, k_att=2.4, k_rep=0.35, d0=0.60, w_gain=2.0):
        self.v_max, self.k_att, self.k_rep = v_max, k_att, k_rep
        self.d0, self.w_gain = d0, w_gain
        self._best = None          # 历史最近目标距离（用于卡住检测）
        self._stuck = 0
        self._escape = 0.0         # 剩余逃逸时间（s）
        self._sign = 1.0

    def reset(self):
        self._best = None
        self._stuck = 0
        self._escape = 0.0
        self._sign = 1.0

    def step(self, pose, scan, goal, dt):
        x, y, yaw = pose
        pts = scan_to_points(pose, scan)
        gx, gy = goal[0] - x, goal[1] - y
        gn = math.hypot(gx, gy) + 1e-9
        force = np.array([self.k_att * gx / gn, self.k_att * gy / gn])
        if len(pts):
            for px, py in pts:
                dx, dy = x - px, y - py
                d = math.hypot(dx, dy)
                if 1e-6 < d < self.d0:
                    mag = self.k_rep * (1.0 / d - 1.0 / self.d0) / (d * d)
                    force += mag * np.array([dx, dy]) / d
        # —— 局部极小值检测：2 s 内没靠近目标就加一个切向分量绕出去 ——
        dist = math.hypot(goal[0] - x, goal[1] - y)
        if self._best is None or dist < self._best - 0.03:
            self._best = dist if self._best is None else min(self._best, dist)
            self._stuck = 0
        else:
            self._stuck += dt
        if self._escape > 0:
            self._escape -= dt
        elif self._stuck > 2.0:
            self._stuck = 0
            self._escape = 1.5
            self._sign = 1.0 if force[1] > 0 else -1.0
        if self._escape > 0 and np.linalg.norm(force) > 1e-9:
            n = force / np.linalg.norm(force)
            tang = np.array([-n[1], n[0]]) * self._sign
            force = force + 1.6 * tang

        if np.linalg.norm(force) < 1e-9:
            return 0.0, 0.0
        want = math.atan2(force[1], force[0])
        err = wrap(want - yaw)
        w = max(-1.5, min(1.5, self.w_gain * err))
        v = self.v_max * max(0.0, math.cos(err))
        return float(v), float(w)


# Bug2（边界跟随）未纳入：其跟墙逻辑按质点设计，0.56 m 足迹在 0.90 m 门道必然刮蹭，
# 属算法前提与场景不匹配。实测数据见 docs 第三节。


PLANNERS = {
    "dwa": DWAPlanner,
    "apf": APFPlanner,
}
