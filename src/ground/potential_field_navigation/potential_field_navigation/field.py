"""人工势场：引力按目标方向，斥力随障碍距离衰减。"""

import math

from . import config


def attractive_force(p, goal, k_att=None):
    """指向目标的单位引力，幅值为 k_att。"""
    k_att = config.K_ATT if k_att is None else k_att
    dx = goal[0] - p[0]
    dy = goal[1] - p[1]
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return (0.0, 0.0)
    return (k_att * dx / d, k_att * dy / d)


def obstacle_repulsion(p, obstacle, eta=None, rho0=None, scale=None):
    """单个圆形障碍的斥力，超出影响半径返回零向量。"""
    eta = config.ETA if eta is None else eta
    rho0 = config.RHO0 if rho0 is None else rho0
    scale = config.REP_SCALE if scale is None else scale
    cx, cy, r = obstacle
    dx = p[0] - cx
    dy = p[1] - cy
    d = math.hypot(dx, dy) - r
    if d <= 1e-6:
        # 贴在障碍表面，沿最近方向给一个强斥力
        nx, ny = dx, dy
        n = math.hypot(nx, ny) or 1.0
        return (scale * nx / n, scale * ny / n)
    if d >= rho0:
        return (0.0, 0.0)
    mag = eta * (1.0 / d - 1.0 / rho0) * scale
    return (mag * dx / math.hypot(dx, dy), mag * dy / math.hypot(dx, dy))


def repulsive_force(p, obstacles=None, **kwargs):
    """所有障碍斥力的合成。"""
    obstacles = config.OBSTACLES if obstacles is None else obstacles
    fx = fy = 0.0
    for ob in obstacles:
        rx, ry = obstacle_repulsion(p, ob, **kwargs)
        fx += rx
        fy += ry
    return (fx, fy)


def total_force(p, goal=None, obstacles=None, **kwargs):
    goal = config.GOAL if goal is None else goal
    obstacles = config.OBSTACLES if obstacles is None else obstacles
    ax, ay = attractive_force(p, goal, kwargs.get("k_att"))
    rx, ry = repulsive_force(p, obstacles, **kwargs)
    return (ax + rx, ay + ry)


def desired_heading(p, goal=None, obstacles=None, bias=None, **kwargs):
    """合力方向；bias 不为空时叠加一个切向分量用于逃出局部极小点。"""
    goal = config.GOAL if goal is None else goal
    obstacles = config.OBSTACLES if obstacles is None else obstacles
    fx, fy = total_force(p, goal, obstacles, **kwargs)
    n = math.hypot(fx, fy)
    if n < 1e-9:
        dx, dy = goal[0] - p[0], goal[1] - p[1]
        n = math.hypot(dx, dy) or 1.0
        fx, fy = dx / n, dy / n
        n = 1.0
    ux, uy = fx / n, fy / n
    if bias:
        # 逆时针切向 = 法向旋转 90 度
        ux, uy = ux - bias * uy, uy + bias * ux
        m = math.hypot(ux, uy) or 1.0
        ux, uy = ux / m, uy / m
    return (ux, uy)
