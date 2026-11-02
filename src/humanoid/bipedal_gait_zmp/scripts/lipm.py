# -*- coding: utf-8 -*-
"""线性倒立摆模型（LIPM）与 ZMP。

把双足机器人简化成一根质量集中在质心、长度等于质心高度的倒立摆，并假设
质心高度恒定。这样水平方向的运动方程是线性的：

    ẍ = ω² (x − p),      ω = sqrt(g / z_c)

x 是质心水平位置，p 是零矩点（ZMP），z_c 是质心高度。

反过来由质心轨迹求 ZMP：

    p = x − ẍ / ω²

几个有用的量：

    自然频率    ω = sqrt(g / z_c)
    捕获点      ξ = x + ẋ / ω
    停止位置    x_stop = x + ẋ / ω

捕获点可以这样理解：如果地面反力只能作用在 ZMP 上，那么质心的"发散分量"
ξ 满足 ξ̇ = ω(ξ − p)。所以要把 ξ 拉回来，ZMP 必须跑到 ξ 前面去。
ξ 落在支撑多边形外时，机器人就站不住了。

给定恒定 ZMP，方程有解析解：

    x(t) = (x₀ − p)·cosh(ωt) + (v₀/ω)·sinh(ωt) + p
    v(t) = (x₀ − p)·ω·sinh(ωt) + v₀·cosh(ωt)
    a(t) = ω²·(x(t) − p)

本模块只用标准库。
"""

from __future__ import annotations

import math

__all__ = [
    "GRAVITY",
    "LIPMParams",
    "omega",
    "com_trajectory",
    "zmp_from_state",
    "capture_point",
    "divergent_com",
    "stop_position",
    "time_to_stop",
]


#: 重力加速度 (m/s²)。用 9.81 与 MuJoCo 默认值一致。
GRAVITY = 9.81


class LIPMParams(object):
    """LIPM 参数。

    z_c  质心高度 (m)，必须为正
    g    重力加速度 (m/s²)
    """

    def __init__(self, z_c=0.8, g=GRAVITY):
        if z_c <= 0:
            raise ValueError("z_c 必须为正")
        if g <= 0:
            raise ValueError("g 必须为正")
        self.z_c = float(z_c)
        self.g = float(g)

    @property
    def omega(self):
        """自然频率 ω = sqrt(g / z_c)，单位 1/s。"""
        return math.sqrt(self.g / self.z_c)

    def __repr__(self):
        return "LIPMParams(z_c=%.3f, g=%.2f, omega=%.3f)" % (
            self.z_c, self.g, self.omega)


def omega(z_c, g=GRAVITY):
    """自然频率 ω = sqrt(g / z_c)。"""
    if z_c <= 0:
        raise ValueError("z_c 必须为正")
    if g <= 0:
        raise ValueError("g 必须为正")
    return math.sqrt(g / z_c)


def com_trajectory(x0, v0, zmp, t, om=None, params=None):
    """恒定 ZMP 下的质心轨迹解析解。

    返回 ``(位置, 速度, 加速度)``。

    x0, v0  初始位置 (m) 与速度 (m/s)
    zmp     恒定 ZMP 位置 (m)
    t       时间 (s)
    om      自然频率；不传就用 ``params`` 算
    """
    if t < 0:
        raise ValueError("t 不能为负")
    if om is None:
        if params is None:
            raise ValueError("om 与 params 至少要给一个")
        om = params.omega
    if om <= 0:
        raise ValueError("omega 必须为正")

    dx = x0 - zmp
    ch = math.cosh(om * t)
    sh = math.sinh(om * t)

    x = dx * ch + (v0 / om) * sh + zmp
    v = dx * om * sh + v0 * ch
    a = om * om * (x - zmp)
    return x, v, a


def zmp_from_state(x, x_ddot, z_c=None, g=GRAVITY):
    """由质心位置与加速度反算 ZMP。

        p = x − ẍ / ω² = x − ẍ · z_c / g
    """
    if z_c is None:
        raise ValueError("需要 z_c")
    if z_c <= 0:
        raise ValueError("z_c 必须为正")
    return x - x_ddot * z_c / g


def capture_point(x, v, om):
    """捕获点 ξ = x + v / ω。

    把脚落在 ξ 上，质心最终能停下来。
    """
    if om <= 0:
        raise ValueError("omega 必须为正")
    return x + v / om


def divergent_com(x, v, om):
    """发散分量，与捕获点数值相同，保留别名便于对照文献叫法。"""
    return capture_point(x, v, om)


def stop_position(x, v, om):
    """当前状态若自由运动到静止，质心最终停在哪里。

    由能量关系导出，等于捕获点。
    """
    return capture_point(x, v, om)


def time_to_stop(x, v, om, zmp=None):
    """质心速度衰减到零所需时间。

    若给定 zmp，按恒定 ZMP 的解析解求解；否则按 zmp = 停止位置
    （即自由减速，速度单调减小）处理。

        v(t) = (x − p)·ω·sinh(ωt) + v·cosh(ωt) = 0

    解得 t = artanh( −v / ((x−p)ω) ) / ω
    """
    if om <= 0:
        raise ValueError("omega 必须为正")
    if zmp is None:
        zmp = stop_position(x, v, om)

    dx = x - zmp
    if abs(dx) < 1e-12:
        # x 正好在 zmp 上，速度按 cosh 增长，永远不会停
        return float("inf")

    arg = -v / (dx * om)
    if abs(arg) >= 1.0:
        return float("inf")
    return math.atanh(arg) / om
