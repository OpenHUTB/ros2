# -*- coding: utf-8 -*-
"""人工势场法（Artificial Potential Field, APF）路径规划核心算法。

基于 Khatib (1986) 提出的反应式规划方法：目标对机器人产生引力、
障碍物产生斥力，机器人沿负梯度（合力）方向逐步步进。

本实现包含三项经典改进：

1. **分段引力势函数**：近场二次（U = ½·k·d²）、远场锥形（U = k·d₀·d - ½·k·d₀²），
   避免远距离引力过大导致步进发散。
2. **障碍物半径建模**：障碍物用圆形 (圆心, 半径) 描述，斥力按机器人到
   障碍物边界的距离计算，更贴近实际物理尺寸。
3. **局部最小值逃逸**：当合力幅值过小或距目标距离持续不下降时，
   注入随机扰动跳出局部势阱。

纯数学层，不依赖 pygame，便于单元测试。
"""

from __future__ import annotations
import math
import random
from collections import namedtuple
from typing import List, Optional, Tuple

Point = Tuple[float, float]

# 障碍物：圆心 (x, y) + 安全半径
Obstacle = namedtuple("Obstacle", ["x", "y", "radius"])


class APFParams:
    """人工势场法参数集合。"""

    def __init__(self,
                 k_att: float = 1.0,
                 k_rep: float = 100.0,
                 d_thresh_att: float = 2.0,
                 d_thresh_rep: float = 1.0,
                 step_size: float = 0.1,
                 max_iter: int = 1000,
                 goal_tol: float = 0.1,
                 escape_jitter: float = 0.3,
                 escape_patience: int = 15):
        self.k_att = k_att
        self.k_rep = k_rep
        self.d_thresh_att = d_thresh_att
        self.d_thresh_rep = d_thresh_rep
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_tol = goal_tol
        self.escape_jitter = escape_jitter
        self.escape_patience = escape_patience

    def validate(self) -> None:
        if self.k_att <= 0:
            raise ValueError("k_att 必须为正")
        if self.k_rep <= 0:
            raise ValueError("k_rep 必须为正")
        if self.d_thresh_att <= 0:
            raise ValueError("d_thresh_att 必须为正")
        if self.d_thresh_rep <= 0:
            raise ValueError("d_thresh_rep 必须为正")
        if self.step_size <= 0:
            raise ValueError("step_size 必须为正")
        if self.max_iter <= 0:
            raise ValueError("max_iter 必须为正")
        if self.goal_tol <= 0:
            raise ValueError("goal_tol 必须为正")


# ---------------------------------------------------------------------------
# 距离
# ---------------------------------------------------------------------------
def distance(a: Point, b: Point) -> float:
    """两点欧氏距离。"""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def obstacle_distance(pos: Point, obs: Obstacle) -> float:
    """机器人到障碍物边界的距离（圆心距减半径）。"""
    return distance(pos, (obs.x, obs.y)) - obs.radius


# ---------------------------------------------------------------------------
# 引力势函数与梯度
# ---------------------------------------------------------------------------
def attractive_potential(pos: Point, goal: Point,
                         k_att: float, d_thresh: float) -> float:
    """引力势函数 U_att。

    近场 (d ≤ d_thresh)：二次型 U = ½·k·d²
    远场 (d > d_thresh)：锥形   U = k·d_thresh·d - ½·k·d_thresh²

    分段保证远距离梯度幅值恒定（= k·d_thresh），避免引力过大。
    """
    d = distance(pos, goal)
    if d <= d_thresh:
        return 0.5 * k_att * d * d
    return k_att * d_thresh * d - 0.5 * k_att * d_thresh * d_thresh


def attractive_gradient(pos: Point, goal: Point,
                        k_att: float, d_thresh: float) -> Point:
    """引力梯度 ∇U_att（指向远离目标方向，正值表示势能增加方向）。

    近场：∇U = k·(pos - goal)
    远场：∇U = k·d_thresh·(pos - goal)/d   （幅值恒定 = k·d_thresh）
    """
    dx = pos[0] - goal[0]
    dy = pos[1] - goal[1]
    d = math.hypot(dx, dy)
    if d < 1e-12:
        return (0.0, 0.0)
    if d <= d_thresh:
        return (k_att * dx, k_att * dy)
    return (k_att * d_thresh * dx / d,
            k_att * d_thresh * dy / d)


# ---------------------------------------------------------------------------
# 斥力势函数与梯度
# ---------------------------------------------------------------------------
def repulsive_potential(pos: Point, obstacles: List[Obstacle],
                        k_rep: float, d_thresh: float) -> float:
    """斥力势函数 U_rep（所有障碍物之和）。

    障碍物影响范围外 (d > d_thresh)：U = 0
    影响范围内 (0 < d ≤ d_thresh)：U = ½·k·(1/d - 1/d_thresh)²
    障碍物内部 (d ≤ 0)：U = +∞
    """
    total = 0.0
    for obs in obstacles:
        d = obstacle_distance(pos, obs)
        if d <= 0.0:
            return float("inf")
        if d <= d_thresh:
            total += 0.5 * k_rep * (1.0 / d - 1.0 / d_thresh) ** 2
    return total


def repulsive_gradient(pos: Point, obstacles: List[Obstacle],
                       k_rep: float, d_thresh: float) -> Point:
    """斥力梯度 ∇U_rep（指向远离障碍物方向）。

    对单个障碍物 (0 < d ≤ d_thresh)：
        ∇U = k·(1/d - 1/d₀)·(1/d²)·(pos - obs_center)/d_center
    其中 d = d_center - radius 是到边界距离，d_center 是到圆心距离。
    """
    gx = 0.0
    gy = 0.0
    for obs in obstacles:
        dx = pos[0] - obs.x
        dy = pos[1] - obs.y
        d_center = math.hypot(dx, dy)
        d = d_center - obs.radius
        if d <= 1e-12:
            # 在障碍物表面或内部：沿远离方向施加极大斥力
            if d_center > 1e-12:
                return (dx / d_center * 1e6, dy / d_center * 1e6)
            return (1e6, 0.0)
        if d <= d_thresh:
            # dU/dd = -k·(1/d - 1/d₀)·(1/d²)  （d 减小时 U 增大，故 dU/dd < 0）
            # dd/d(pos) = (pos - obs) / d_center
            # ∇U_rep = dU/dd · dd/d(pos) = -k·(1/d-1/d₀)/(d²·d_center)·(pos-obs)
            # 指向障碍物方向（势能增加方向），合力取负后即远离障碍物
            factor = -k_rep * (1.0 / d - 1.0 / d_thresh) / (d * d * d_center)
            gx += factor * dx
            gy += factor * dy
    return (gx, gy)


# ---------------------------------------------------------------------------
# 合力（负梯度）
# ---------------------------------------------------------------------------
def total_force(pos: Point, goal: Point,
                obstacles: List[Obstacle],
                params: APFParams) -> Point:
    """合力 F = -∇U_att - ∇U_rep（机器人受力方向）。"""
    ax, ay = attractive_gradient(pos, goal, params.k_att, params.d_thresh_att)
    rx, ry = repulsive_gradient(pos, obstacles, params.k_rep, params.d_thresh_rep)
    return (-ax - rx, -ay - ry)


# ---------------------------------------------------------------------------
# 单步迭代
# ---------------------------------------------------------------------------
def step(pos: Point, goal: Point,
         obstacles: List[Obstacle],
         params: APFParams) -> Tuple[Point, Point]:
    """沿合力方向走一个步长。返回 (new_pos, force)。

    合力归一化为单位向量后乘 step_size，保证步长恒定不发散。
    """
    fx, fy = total_force(pos, goal, obstacles, params)
    fmag = math.hypot(fx, fy)
    if fmag < 1e-12:
        return (pos[0], pos[1]), (fx, fy)
    nx = pos[0] + params.step_size * fx / fmag
    ny = pos[1] + params.step_size * fy / fmag
    return (nx, ny), (fx, fy)


# ---------------------------------------------------------------------------
# 完整规划
# ---------------------------------------------------------------------------
def plan(start: Point, goal: Point,
         obstacles: List[Obstacle],
         params: APFParams,
         rng: Optional[random.Random] = None
         ) -> Tuple[List[Point], str]:
    """从 start 规划到 goal，返回 (路径点列表, 终止状态)。

    终止状态：
        'reached'  : 到达目标容差范围内
        'timeout'  : 达到最大迭代次数仍未到达
        'diverged' : 位置发散（坐标绝对值超过 1e4）
        'trapped'  : 陷入障碍物内部
    """
    params.validate()
    if rng is None:
        rng = random.Random(0)

    pos = (float(start[0]), float(start[1]))
    path: List[Point] = [pos]
    no_progress = 0
    last_dist = distance(pos, goal)

    for _ in range(params.max_iter):
        # 到达判定
        if distance(pos, goal) <= params.goal_tol:
            return path, "reached"

        # 陷入障碍物判定
        for obs in obstacles:
            if obstacle_distance(pos, obs) <= 0.0:
                return path, "trapped"

        new_pos, force = step(pos, goal, obstacles, params)
        fmag = math.hypot(*force)

        # 局部最小值检测：合力过小 或 距目标不下降
        new_dist = distance(new_pos, goal)
        stuck = (fmag < 1e-6) or (new_dist >= last_dist - 1e-9)

        if stuck:
            no_progress += 1
            if no_progress > params.escape_patience:
                # 随机扰动逃逸
                jx = rng.uniform(-1.0, 1.0) * params.escape_jitter
                jy = rng.uniform(-1.0, 1.0) * params.escape_jitter
                new_pos = (pos[0] + jx, pos[1] + jy)
                no_progress = 0
        else:
            no_progress = 0

        last_dist = distance(new_pos, goal)
        path.append(new_pos)
        pos = new_pos

        # 发散检测
        if abs(pos[0]) > 1e4 or abs(pos[1]) > 1e4:
            return path, "diverged"

    # 末尾再判一次到达
    if distance(pos, goal) <= params.goal_tol:
        return path, "reached"
    return path, "timeout"


# ---------------------------------------------------------------------------
# 势场采样（可视化用）
# ---------------------------------------------------------------------------
def potential_at(pos: Point, goal: Point,
                 obstacles: List[Obstacle],
                 params: APFParams) -> float:
    """某点总势能 U = U_att + U_rep（可视化热图用）。"""
    u_att = attractive_potential(pos, goal, params.k_att, params.d_thresh_att)
    u_rep = repulsive_potential(pos, obstacles, params.k_rep, params.d_thresh_rep)
    return u_att + u_rep
