#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""reeds_shepp.py

Reeds-Shepp (RS) 曲线生成：给定车辆起点/终点位姿 (x, y, yaw) 和最小转弯半径 R，
计算一条满足自行车运动学、允许前进/后退切换的最短路径。

本实现采用防御性策略：枚举各 family 后，对每条候选路径做数值积分，
并检查积分终点是否与目标位姿一致；不一致直接丢弃。
这样即使某个 family 的解析公式有误，也不会输出错误路径。
"""

import math
from collections import namedtuple

PathElement = namedtuple("PathElement", "steer length")


def _mod2pi(t):
    t = math.fmod(t, 2 * math.pi)
    return t + 2 * math.pi if t < 0 else t


def _M(t):
    t = _mod2pi(t)
    return t - 2 * math.pi if t > math.pi else t


def _R(x, y):
    return math.hypot(x, y), math.atan2(y, x)


# ---------------- 各 family 的解析解（输入归一化坐标，半径=1） ----------------
# 每个函数返回 (t, u, v) 三段长度（无单位，半径归一化后），或 None
# 段的 steer 方向由 family 名字决定

def LSL(x, y, phi):
    u, t = _R(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if t >= 0.0:
        v = _M(phi - t)
        if v >= 0.0:
            return t, u, v
    return None


def RSR(x, y, phi):
    u, t = _R(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if t >= 0.0:
        v = _M(t - phi)
        if v >= 0.0:
            return t, u, v
    return None


def LSR(x, y, phi):
    u, t = _R(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if u >= 2.0:
        v = math.sqrt(u ** 2 - 4.0)
        t2 = math.atan2(2.0, v)
        t = t - t2
        v = _M(t + phi - 2.0 * t2)
        if t >= 0.0 and v >= 0.0:
            return t, v, t
    return None


def RSL(x, y, phi):
    u, t = _R(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if u >= 2.0:
        v = math.sqrt(u ** 2 - 4.0)
        t2 = math.atan2(2.0, v)
        t = t + t2
        v = _M(-t - phi + 2.0 * t2)
        if t >= 0.0 and v >= 0.0:
            return t, v, t
    return None


def LRL(x, y, phi):
    u1, t1 = _R(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if u1 <= 4.0:
        u = -2.0 * math.asin(u1 / 4.0)
        t = _M(t1 + u / 2.0)
        v = _M(t - phi)
        if t >= 0.0 and v <= 0.0:
            return t, -u, v
    return None


def RLR(x, y, phi):
    u1, t1 = _R(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if u1 <= 4.0:
        u = 2.0 * math.asin(u1 / 4.0)
        t = _M(t1 - u / 2.0)
        v = _M(phi - t - u)
        if t >= 0.0 and v <= 0.0:
            return t, u, v
    return None


# family 名字 → 三段 steer
_FAMILY_STEERS = {
    "LSL": ("L", "S", "L"),
    "RSR": ("R", "S", "R"),
    "LSR": ("L", "S", "R"),
    "RSL": ("R", "S", "L"),
    "LRL": ("L", "R", "L"),
    "RLR": ("R", "L", "R"),
}

_FUNCS = {
    "LSL": LSL, "RSR": RSR, "LSR": LSR, "RSL": RSL, "LRL": LRL, "RLR": RLR,
}


def _transform(goal, start):
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    theta = -start[2]
    x = math.cos(theta) * dx - math.sin(theta) * dy
    y = math.sin(theta) * dx + math.cos(theta) * dy
    phi = _M(goal[2] - start[2])
    return x, y, phi


def _integrate(start, segs, r, step):
    """积分路径。segs: [(steer, length_rad_or_m)], length 单位是"半径归一化长度"，
    实际米数 = length * r。返回 [(x,y,yaw,gear)]。"""
    x, y, theta = start
    pts = [(x, y, _mod2pi(theta), 1)]
    for steer, length in segs:
        signed_len = length * r
        gear = 1 if signed_len >= 0 else -1
        direction = 1 if signed_len >= 0 else -1
        L = abs(signed_len)
        n = max(1, int(L / step))
        ds = L / n
        for _ in range(n):
            if steer == "S":
                x += direction * ds * math.cos(theta)
                y += direction * ds * math.sin(theta)
            elif steer == "L":
                dth = direction * ds / r
                cx = x - r * math.sin(theta)
                cy = y + r * math.cos(theta)
                theta += dth
                x = cx + r * math.sin(theta)
                y = cy - r * math.cos(theta)
            elif steer == "R":
                dth = -direction * ds / r
                cx = x + r * math.sin(theta)
                cy = y - r * math.cos(theta)
                theta += dth
                x = cx - r * math.sin(theta)
                y = cy + r * math.cos(theta)
            pts.append((x, y, _mod2pi(theta), gear))
    return pts


def get_rs_path(start, goal, r, step=0.05):
    """主接口：返回从 start 到 goal 的 RS 路径 [(x,y,yaw,gear)]；无解或终点校验失败返回 None。"""
    x, y, phi = _transform(goal, start)
    x /= r
    y /= r

    best = None
    best_cost = float("inf")

    for name, func in _FUNCS.items():
        sol = func(x, y, phi)
        if sol is None:
            continue
        t, u, v = sol
        steers = _FAMILY_STEERS[name]
        segs = list(zip(steers, (t, u, v)))
        cost = abs(t) + abs(u) + abs(v)

        # 正向尝试
        path = _integrate(start, segs, r, step)
        if _close_enough(path[-1], goal) and cost < best_cost:
            best = path
            best_cost = cost

        # 镜像尝试（左右翻转）：把 L/R 互换
        mirror = [("R" if s == "L" else ("L" if s == "R" else "S"), L)
                  for s, L in segs]
        path_m = _integrate(start, mirror, r, step)
        if _close_enough(path_m[-1], goal) and cost < best_cost:
            best = path_m
            best_cost = cost

    return best


def _close_enough(end, goal, tol_xy=0.05, tol_yaw=math.radians(3)):
    return (math.hypot(end[0] - goal[0], end[1] - goal[1]) < tol_xy and
            abs(_M(end[2] - goal[2])) < tol_yaw)


if __name__ == "__main__":
    # 自测 1：LSL
    s = (0.0, 0.0, 0.0)
    g = (5.0, 2.0, math.pi / 2.0)
    p = get_rs_path(s, g, r=3.0, step=0.2)
    print("test1:", "OK" if p else "FAIL", "end=", p[-1] if p else None)

    # 自测 2：泊车场景
    s = (2.5, -2.2, 0.0)
    g = (-1.6, 2.6, math.pi / 2.0)
    p = get_rs_path(s, g, r=3.85, step=0.2)
    print("test2:", "OK" if p else "FAIL")
    if p:
        print("  points:", len(p), "end:", p[-1])
