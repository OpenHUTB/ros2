# -*- coding: utf-8 -*-
"""Dubins 曲线核心算法（几何构造法）。

给定起点与终点位姿 (x, y, yaw) 及最小转弯半径 R，求带曲率约束的最短可行路径。

基于 Lester E. Dubins (1957) 的经典结论：最短路径必为以下 6 种组合之一：
    CSC 类型：LSL / LSR / RSL / RSR  （圆-直线-圆）
    CCC 类型：LRL / RLR              （圆-圆-圆）

其中 L = 左转（逆时针圆弧）、R = 右转（顺时针圆弧）、S = 直线段。

本实现采用几何构造法：计算起点/终点的左右转圆心，再分别求 6 种组合的
公切线（CSC）或中间相切圆（CCC），直接得到 3 段弧长 t/p/q。相比闭式公式，
几何法直观、易验证，且数值稳定性好。

纯数学层，不依赖 pygame，便于单元测试。
"""

from __future__ import annotations
import math
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]
Pose = Tuple[float, float, float]  # (x, y, yaw)


# ---------------------------------------------------------------------------
# 角度归一化
# ---------------------------------------------------------------------------
def mod2pi(theta: float) -> float:
    """将 theta 归一化到 [0, 2π)。"""
    return theta - 2.0 * math.pi * math.floor(theta / (2.0 * math.pi))


# ---------------------------------------------------------------------------
# 圆心计算
# ---------------------------------------------------------------------------
def _left_center(pose: Pose, R: float) -> Point:
    """左转圆心（圆心在位姿朝向的左侧，+π/2 方向）。"""
    x, y, yaw = pose
    return (x - R * math.sin(yaw), y + R * math.cos(yaw))


def _right_center(pose: Pose, R: float) -> Point:
    """右转圆心（圆心在位姿朝向的右侧，-π/2 方向）。"""
    x, y, yaw = pose
    return (x + R * math.sin(yaw), y - R * math.cos(yaw))


def _angle_on_circle(point: Point, center: Point) -> float:
    """point 在以 center 为圆心的圆上的角度（相对圆心）。"""
    return math.atan2(point[1] - center[1], point[0] - center[0])


# ---------------------------------------------------------------------------
# 弧长计算
# ---------------------------------------------------------------------------
def _arc_length(a_from: float, a_to: float, turn: str) -> float:
    """从角度 a_from 到 a_to 沿 turn 方向的弧长（归一化，半径=1）。

    turn='L': 逆时针（角度递增）
    turn='R': 顺时针（角度递减）
    返回值 ∈ [0, 2π)
    """
    if turn == "L":
        return mod2pi(a_to - a_from)
    else:  # 'R'
        return mod2pi(a_from - a_to)


# ---------------------------------------------------------------------------
# CSC 路径（圆-直线-圆）
# ---------------------------------------------------------------------------
def _external_tangent(c1: Point, c2: Point, R: float, turn: str
                      ) -> Optional[Tuple[Point, Point, float]]:
    """两等径圆（半径 R）的外公切线（用于 LSL/RSR）。

    返回 (切点1, 切点2, 切线方向)，无解（圆心重合）时返回 None。
    外公切线方向 = 圆心连线方向（等径圆）。
    切点在圆上的位置：
      L（左转圆）：切点角度 = 方向 - π/2（机器人在圆心右侧）
      R（右转圆）：切点角度 = 方向 + π/2（机器人在圆心左侧）
    """
    dx = c2[0] - c1[0]
    dy = c2[1] - c1[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-12:
        return None
    direction = math.atan2(dy, dx)
    offset = -math.pi / 2.0 if turn == "L" else math.pi / 2.0
    p1 = (c1[0] + R * math.cos(direction + offset),
          c1[1] + R * math.sin(direction + offset))
    p2 = (c2[0] + R * math.cos(direction + offset),
          c2[1] + R * math.sin(direction + offset))
    return p1, p2, direction


def _internal_tangent(c1: Point, c2: Point, R: float, turn1: str, turn2: str
                      ) -> Optional[Tuple[Point, Point, float]]:
    """两等径圆（半径 R）的内公切线（用于 LSR/RSL）。

    turn1 是 c1 的转向，turn2 是 c2 的转向（两者相反）。内公切线要求圆心距 >= 2R。

    推导：设切线方向为 dir，n 为 dir 的左法向 (dir + π/2)，
    切点按转向落在圆心两侧：L 圆取 -n 侧、R 圆取 +n 侧，即
        p1 = c1 + R·(dir - π/2)   (turn1='L')      p1 = c1 + R·(dir + π/2)  (turn1='R')
        p2 = c2 - R·(dir - π/2)   (turn2='R')      p2 = c2 + R·(dir - π/2)  (turn2='L')
    于是 p2 - p1 = (c2 - c1) + s·2R·n，其中 s=+1 (L,R)、s=-1 (R,L)。
    要求 p2 - p1 ∥ dir，即 (p2-p1)·n = 0：

        dist·sin(base - dir) + s·2R = 0  →  sin(dir - base) = s·2R/dist

    两支解 dir = base + a 与 dir = base + π - a 都满足正弦条件，
    但只有 cos(dir - base) > 0 的那支让直线段沿 +dir 前进（另一支是反向，
    会让 p 变成负值），所以取 dir = base + a，a = asin(s·2R/dist)。

    原先的实现固定用 base+φ / base+π-φ 两支，对 RSL（s=-1）一支都不满足，
    导致 RSL 从不出现、shortest_path 退而选更长的路径。
    """
    dx = c2[0] - c1[0]
    dy = c2[1] - c1[1]
    dist = math.hypot(dx, dy)
    if dist < 2.0 * R - 1e-9 or dist < 1e-12:
        return None

    base = math.atan2(dy, dx)
    sign = 1.0 if turn1 == "L" else -1.0          # (L,R) -> +1, (R,L) -> -1
    ratio = max(-1.0, min(1.0, sign * 2.0 * R / dist))
    direction = base + math.asin(ratio)

    off1 = -math.pi / 2.0 if turn1 == "L" else math.pi / 2.0
    off2 = -math.pi / 2.0 if turn2 == "L" else math.pi / 2.0
    p1 = (c1[0] + R * math.cos(direction + off1),
          c1[1] + R * math.sin(direction + off1))
    p2 = (c2[0] + R * math.cos(direction + off2),
          c2[1] + R * math.sin(direction + off2))

    # 数值校验：切点连线必须与 dir 同向
    vdx = p2[0] - p1[0]
    vdy = p2[1] - p1[1]
    if math.hypot(vdx, vdy) < 1e-9:
        return None
    delta = abs(mod2pi(math.atan2(vdy, vdx) - direction + math.pi) - math.pi)
    if delta > 1e-6:
        return None
    return p1, p2, direction


def _csc_path(start: Pose, end: Pose, R: float, turn1: str, turn2: str
              ) -> Optional[Tuple[float, float, float]]:
    """计算 CSC 路径的 (t, p, q) 实际长度。

    turn1, turn2 ∈ {'L', 'R'}，中间段为 'S'。
    """
    if turn1 == "L":
        c1 = _left_center(start, R)
    else:
        c1 = _right_center(start, R)
    if turn2 == "L":
        c2 = _left_center(end, R)
    else:
        c2 = _right_center(end, R)

    if turn1 == turn2:
        # 外公切线（LSL 或 RSR）
        tan_info = _external_tangent(c1, c2, R, turn1)
    else:
        # 内公切线（LSR 或 RSL）
        tan_info = _internal_tangent(c1, c2, R, turn1, turn2)

    if tan_info is None:
        return None
    p1, p2, direction = tan_info

    # 起点在 c1 圆上的角度
    a_start = _angle_on_circle(start, c1)
    a_p1 = _angle_on_circle(p1, c1)
    t = _arc_length(a_start, a_p1, turn1) * R

    # 直线段长度
    p = math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    # 终点在 c2 圆上的角度
    a_end = _angle_on_circle(end, c2)
    a_p2 = _angle_on_circle(p2, c2)
    q = _arc_length(a_p2, a_end, turn2) * R

    return (t, p, q)


# ---------------------------------------------------------------------------
# CCC 路径（圆-圆-圆）
# ---------------------------------------------------------------------------
def _ccc_path(start: Pose, end: Pose, R: float, turn1: str
              ) -> Optional[Tuple[float, float, float]]:
    """计算 CCC 路径的 (t, p, q) 实际长度。

    turn1='L' → LRL，turn1='R' → RLR。
    中间圆与两个主圆外切（半径同为 R），圆心落在 c1-c2 连线的中垂线上，
    有 ± 两个候选；两个候选都是几何上可行的闭链，但总长通常不同，
    必须都算出来取较短的那个。

    原先的实现遇到第一个满足距离条件的中间圆就直接 return，
    于是 LRL/RLR 可能返回较长解，shortest_path 随之选错类型。
    """
    turn_mid = "R" if turn1 == "L" else "L"
    turn3 = turn1

    if turn1 == "L":
        c1 = _left_center(start, R)
        c2 = _left_center(end, R)
    else:
        c1 = _right_center(start, R)
        c2 = _right_center(end, R)

    dx = c2[0] - c1[0]
    dy = c2[1] - c1[1]
    dist = math.hypot(dx, dy)
    # 中间圆与两主圆外切 → 圆心距均为 2R，因此要求 dist <= 4R
    if dist > 4.0 * R - 1e-9 or dist < 1e-12:
        return None

    base = math.atan2(dy, dx)
    h = math.sqrt(max(0.0, (2.0 * R) ** 2 - (dist / 2.0) ** 2))
    perp = base + math.pi / 2.0
    mid = ((c1[0] + c2[0]) / 2.0, (c1[1] + c2[1]) / 2.0)

    best = None
    for sign in (1.0, -1.0):
        cm = (mid[0] + sign * h * math.cos(perp),
              mid[1] + sign * h * math.sin(perp))
        d1m = math.hypot(cm[0] - c1[0], cm[1] - c1[1])
        d2m = math.hypot(cm[0] - c2[0], cm[1] - c2[1])
        if abs(d1m - 2.0 * R) > 1e-6 or abs(d2m - 2.0 * R) > 1e-6:
            continue
        # 切点 1：c1 与 cm 的外切点（沿 c1→cm 方向距 c1 为 R）
        tan1 = (c1[0] + R * (cm[0] - c1[0]) / d1m,
                c1[1] + R * (cm[1] - c1[1]) / d1m)
        # 切点 2：cm 与 c2 的外切点（沿 cm→c2 方向距 cm 为 R）
        tan2 = (cm[0] + R * (c2[0] - cm[0]) / d2m,
                cm[1] + R * (c2[1] - cm[1]) / d2m)

        a_start = _angle_on_circle(start, c1)
        a_t1 = _angle_on_circle(tan1, c1)
        t = _arc_length(a_start, a_t1, turn1) * R

        a_t1m = _angle_on_circle(tan1, cm)
        a_t2m = _angle_on_circle(tan2, cm)
        p = _arc_length(a_t1m, a_t2m, turn_mid) * R

        a_t2 = _angle_on_circle(tan2, c2)
        a_end = _angle_on_circle(end, c2)
        q = _arc_length(a_t2, a_end, turn3) * R

        total = t + p + q
        if best is None or total < best[0] - 1e-12:
            best = (total, (t, p, q))
    return best[1] if best is not None else None


# ---------------------------------------------------------------------------
# 6 种组合
# ---------------------------------------------------------------------------
DUBINS_TYPES = ["LSL", "RSR", "LSR", "RSL", "LRL", "RLR"]


def _pose_equal(a: Pose, b: Pose, tol: float = 1e-9) -> bool:
    """两个位姿是否相同（位置与朝向都相同，朝向按 2π 周期比较）。"""
    if math.hypot(a[0] - b[0], a[1] - b[1]) > tol:
        return False
    dyaw = abs(mod2pi(a[2] - b[2] + math.pi) - math.pi)
    return dyaw <= tol


def _compute_path(start: Pose, end: Pose, R: float, path_type: str
                  ) -> Optional[Tuple[float, float, float]]:
    """计算指定类型路径的实际长度 (t, p, q)。

    起末位姿完全相同时，六种组合都退化为零长度路径，返回 (0, 0, 0)；
    否则起点终点重合时下面各构造会因圆心重合而全部返回 None，
    shortest_path 会错误地抛 ValueError。
    """
    if _pose_equal(start, end):
        return (0.0, 0.0, 0.0)
    if path_type == "LSL":
        return _csc_path(start, end, R, "L", "L")
    if path_type == "RSR":
        return _csc_path(start, end, R, "R", "R")
    if path_type == "LSR":
        return _csc_path(start, end, R, "L", "R")
    if path_type == "RSL":
        return _csc_path(start, end, R, "R", "L")
    if path_type == "LRL":
        return _ccc_path(start, end, R, "L")
    if path_type == "RLR":
        return _ccc_path(start, end, R, "R")
    raise ValueError(f"未知路径类型: {path_type}")


# ---------------------------------------------------------------------------
# 求最短 Dubins 路径
# ---------------------------------------------------------------------------
def shortest_path(start: Pose, end: Pose, radius: float
                  ) -> Tuple[str, Tuple[float, float, float], float]:
    """求从 start 到 end 的最短 Dubins 路径。

    Parameters
    ----------
    start : (x0, y0, yaw0) 起点位姿
    end : (x1, y1, yaw1) 终点位姿
    radius : 最小转弯半径（>0）

    Returns
    -------
    (best_type, (t, p, q), total_length)
        best_type : 6 种组合之一
        (t, p, q) : 三段实际长度
        total_length : 路径总长度
    """
    if radius <= 0:
        raise ValueError("radius 必须为正")

    best_type = None
    best_tpq = None
    best_cost = float("inf")
    for pt in DUBINS_TYPES:
        res = _compute_path(start, end, radius, pt)
        if res is None:
            continue
        t, p, q = res
        cost = t + p + q
        if cost < best_cost - 1e-12:
            best_cost = cost
            best_type = pt
            best_tpq = (t, p, q)

    if best_type is None:
        raise ValueError("无法找到 Dubins 路径（数值退化）")

    return best_type, best_tpq, best_cost


def all_paths(start: Pose, end: Pose, radius: float
              ) -> List[Tuple[str, Tuple[float, float, float], float]]:
    """返回所有 6 种组合的长度（用于可视化对比），按总长度升序。"""
    if radius <= 0:
        raise ValueError("radius 必须为正")
    out = []
    for pt in DUBINS_TYPES:
        res = _compute_path(start, end, radius, pt)
        if res is None:
            continue
        t, p, q = res
        out.append((pt, (t, p, q), t + p + q))
    out.sort(key=lambda x: x[2])
    return out


# ---------------------------------------------------------------------------
# 沿路径采样点
# ---------------------------------------------------------------------------
def sample_path(start: Pose, radius: float, tpq: Tuple[float, float, float],
                path_type: str, n: int = 200) -> List[Pose]:
    """沿给定 Dubins 路径采样 n 个位姿点。"""
    if path_type not in DUBINS_TYPES:
        raise ValueError(f"未知路径类型: {path_type}")
    t, p, q = tpq
    total = t + p + q
    if total < 1e-12 or n < 1:
        return [start]
    if n == 1:
        return [_endpoint(start, radius, tpq, path_type)]

    points: List[Pose] = []
    for i in range(n):
        s = total * i / (n - 1)
        points.append(_point_at_arclength(start, radius, t, p, q, path_type, s))
    return points


def _point_at_arclength(start: Pose, radius: float, t: float, p: float, q: float,
                        path_type: str, s: float) -> Pose:
    """沿路径累计弧长 s 处的位姿。"""
    x, y, yaw = start
    s1, s2, s3 = path_type[0], path_type[1], path_type[2]
    if s <= t:
        return _advance(x, y, yaw, radius, s1, s)
    s -= t
    x, y, yaw = _advance(x, y, yaw, radius, s1, t)
    if s <= p:
        return _advance(x, y, yaw, radius, s2, s)
    s -= p
    x, y, yaw = _advance(x, y, yaw, radius, s2, p)
    return _advance(x, y, yaw, radius, s3, s)


def _advance(x: float, y: float, yaw: float, radius: float,
             seg: str, length: float) -> Pose:
    """从当前位姿沿 seg 类型走 length 距离后的新位姿。

    L = 左转（逆时针），R = 右转（顺时针），S = 直线。
    圆弧公式（几何推导，R 段圆心在右侧）：
        L: dx = R*(sin(yaw+d) - sin(yaw));    dy = R*(cos(yaw) - cos(yaw+d));  dyaw = +d
        R: dx = R*(sin(yaw) - sin(yaw-d));    dy = R*(cos(yaw-d) - cos(yaw));  dyaw = -d
    其中 d = length / radius（R 转弧长 / 半径）。
    """
    if length == 0.0:
        return (x, y, yaw)
    if seg == "S":
        return (x + length * math.cos(yaw),
                y + length * math.sin(yaw),
                yaw)
    d = length / radius
    if seg == "L":
        nx = x + radius * (math.sin(yaw + d) - math.sin(yaw))
        ny = y + radius * (math.cos(yaw) - math.cos(yaw + d))
        return (nx, ny, mod2pi(yaw + d))
    else:  # seg == "R"
        nx = x + radius * (math.sin(yaw) - math.sin(yaw - d))
        ny = y + radius * (math.cos(yaw - d) - math.cos(yaw))
        return (nx, ny, mod2pi(yaw - d))


def _endpoint(start: Pose, radius: float, tpq: Tuple[float, float, float],
              path_type: str) -> Pose:
    """整条路径终点位姿。"""
    t, p, q = tpq
    x, y, yaw = start
    s1, s2, s3 = path_type[0], path_type[1], path_type[2]
    x, y, yaw = _advance(x, y, yaw, radius, s1, t)
    x, y, yaw = _advance(x, y, yaw, radius, s2, p)
    x, y, yaw = _advance(x, y, yaw, radius, s3, q)
    return (x, y, yaw)
