# -*- coding: utf-8 -*-
"""车辆动力学指标计算（纯函数，不依赖 CARLA）。

这里的函数只做数学计算，因此可以脱离 CARLA 单独做单元测试
（见 ``test/test_metrics.py``），这也是本模块"离线可自证"的基础。

符号约定
--------
* 速度单位 m/s，距离 m，时间 s，角度 rad；
* ``v0`` / ``v1`` 为测试区间的起止速度；
* 车辆纵向加速度 ``a = dv/dt``，减速时取正值表示"减速度大小"。
"""

import math

import numpy as np


def mean_acceleration(v0, v1, dt):
    """区间平均加速度 a = (v1 - v0) / dt。

    >>> round(mean_acceleration(0.0, 16.67, 10.0), 3)
    1.667
    """
    dt = float(dt)
    if dt <= 0.0:
        raise ValueError('时间间隔必须为正数')
    return (float(v1) - float(v0)) / dt


def braking_distance_theory(v, decel):
    """匀减速假设下的理论制动距离 s = v^2 / (2a)。

    >>> round(braking_distance_theory(16.67, 6.0), 2)
    23.16
    """
    decel = float(decel)
    if decel <= 0.0:
        raise ValueError('减速度必须为正数')
    return float(v) ** 2 / (2.0 * decel)


def stopping_deceleration(v, distance):
    """由实测制动距离反推平均减速度 a = v^2 / (2s)。"""
    distance = float(distance)
    if distance <= 0.0:
        raise ValueError('制动距离必须为正数')
    return float(v) ** 2 / (2.0 * distance)


def turn_radius_theory(wheelbase, steer):
    """阿克曼自行车模型的稳态转弯半径 R = L / tan(delta)。

    注意：CARLA 的 ``VehicleControl.steer`` 是**归一化的无量纲量**（-1~1），
    需要先乘以该车的最大前轮转角 ``max_steer``（弧度）才是真实的 delta，
    即 ``delta = steer * max_steer``（见 :func:`steer_to_angle`）。
    """
    delta = abs(float(steer))
    if delta <= 1e-9:
        raise ValueError('前轮转角不能为 0（直线行驶没有转弯半径）')
    return float(wheelbase) / math.tan(delta)


def steer_to_angle(steer, max_steer_rad):
    """把归一化转向指令换算成真实前轮转角（弧度）。"""
    return float(steer) * float(max_steer_rad)


def circle_fit_radius(points):
    """用 Kasa 最小二乘法拟合圆，返回 (半径, 圆心)。

    给定 n 个二维点 ``(x, y)``，拟合最贴近的圆。做法是最小化
    ``sum((x^2 + y^2) - (2*x0*x + 2*y0*y + (r^2 - x0^2 - y0^2)))^2``，
    即把问题化成线性最小二乘 ``A * [x0, y0, c] = b``，其中
    ``b = x^2 + y^2``、``c = r^2 - x0^2 - y0^2``。

    ``points`` 少于 3 个点或退化（共线）时抛 :class:`ValueError`。
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError('points 必须是 (n, 2) 的二维坐标数组')
    if pts.shape[0] < 3:
        raise ValueError('至少需要 3 个点才能拟合圆')

    x, y = pts[:, 0], pts[:, 1]
    a = np.column_stack((2.0 * x, 2.0 * y, np.ones_like(x)))
    b = x ** 2 + y ** 2
    solution, residuals, rank, _ = np.linalg.lstsq(a, b, rcond=None)
    if rank < 3:
        raise ValueError('点共线或重合，无法拟合出圆')
    x0, y0, c = solution
    squared = c + x0 ** 2 + y0 ** 2
    if squared <= 0.0:
        raise ValueError('拟合结果退化（半径平方为负）')
    return float(math.sqrt(squared)), (float(x0), float(y0))


def radius_from_yaw_rate(times, yaws, speeds):
    """由稳态偏航角速度求转弯半径：``R = v / omega``。

    ``omega`` 用最小二乘直线拟合 ``yaw(t)`` 的斜率得到（比"首尾差分"更抗噪声），
    并对偏航角做 ``unwrap`` 处理，避免 ±π 处跳变。

    返回 ``(半径, 角速度, 平均车速)``；角速度接近 0（直线行驶）时半径为 ``None``。
    """
    times = np.asarray(times, dtype=np.float64)
    yaws = np.unwrap(np.asarray(yaws, dtype=np.float64))
    speeds = np.asarray(speeds, dtype=np.float64)
    if times.size < 3:
        raise ValueError('至少需要 3 个采样点才能估计偏航角速度')
    omega = float(np.polyfit(times, yaws, 1)[0])
    v_mean = float(np.mean(speeds))
    if abs(omega) < 1e-6:
        return None, omega, v_mean
    return v_mean / abs(omega), omega, v_mean


def path_length(points):
    """折线总长度（用于核对制动距离等）。"""
    pts = np.asarray(points, dtype=np.float64)
    if pts.shape[0] < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def summarize_test(values, name):
    """把一组测试值汇总成 min/mean/max 字典（用于打印对比表）。"""
    arr = np.asarray([v for v in values if v is not None and np.isfinite(v)],
                     dtype=np.float64)
    if arr.size == 0:
        return {'name': name, 'n': 0, 'min': None, 'mean': None, 'max': None}
    return {'name': name, 'n': int(arr.size),
            'min': float(arr.min()), 'mean': float(arr.mean()), 'max': float(arr.max())}
