# -*- coding: utf-8 -*-
"""水面无人艇（USV）视线（LOS）路径跟踪制导（只用 Python 标准库）。

运动学模型（恒速、欠驱动只考虑航向）：
    x_dot = U * cos(psi)
    y_dot = U * sin(psi)
    psi_dot = K * wrap(chi_d - psi)        # 一阶航向跟踪

LOS 制导律（针对当前路径段 P1->P2）：
    把船位置投影到路径坐标系，得沿迹 s 与垂迹（横漂）y_e；
    期望航向 chi_d = chi_p + atan2(-y_e, Delta)
其中 chi_p 为路径切线角，Delta 为前视距离。y_e 越大，期望航向越偏向路径，
从而把船拉回航线。
"""

from __future__ import annotations

import math


def wrap(a):
    """把角度归一化到 [-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def los_guidance(pos, p1, p2, delta):
    """返回 (期望航向 chi_d, 横漂误差 y_e)。"""
    x, y = pos
    chi_p = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    dx = x - p1[0]
    dy = y - p1[1]
    # 路径坐标系下的垂迹误差（左正）
    y_e = -dx * math.sin(chi_p) + dy * math.cos(chi_p)
    chi_d = chi_p + math.atan2(-y_e, delta)
    return chi_d, y_e


def simulate(waypoints, start, U=1.5, K=1.2, delta=3.0, dt=0.05,
             reach_radius=0.6, max_t=200.0):
    """沿 waypoints 做 LOS 路径跟踪，返回 (轨迹点列表, 横漂序列, 时间序列)。"""
    if len(waypoints) < 2:
        raise ValueError("至少需要两个航点")
    pos = [float(start[0]), float(start[1])]
    psi = 0.0
    seg = 0
    t = 0.0
    traj = [tuple(pos)]
    cte = [los_guidance(pos, waypoints[0], waypoints[1], delta)[1]]
    ts = [0.0]
    while seg < len(waypoints) - 1:
        p1 = waypoints[seg]
        p2 = waypoints[seg + 1]
        chi_d, y_e = los_guidance(pos, p1, p2, delta)
        e = wrap(chi_d - psi)
        psi += K * e * dt
        pos[0] += U * math.cos(psi) * dt
        pos[1] += U * math.sin(psi) * dt
        t += dt
        traj.append((pos[0], pos[1]))
        cte.append(y_e)
        ts.append(t)
        # 到达当前航点 -> 进入下一段
        if math.hypot(pos[0] - p2[0], pos[1] - p2[1]) < reach_radius:
            seg += 1
        if t > max_t:
            break
    return traj, cte, ts


if __name__ == "__main__":
    wps = [(0.0, 0.0), (12.0, 0.0), (12.0, 8.0)]
    traj, cte, _ = simulate(wps, (0.0, 3.0))
    print("末位置: (%.2f, %.2f)" % traj[-1])
    print("最终横漂: %.4f" % abs(cte[-1]))
