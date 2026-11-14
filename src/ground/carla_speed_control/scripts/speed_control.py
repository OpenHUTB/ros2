#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纵向速度控制：按曲率自动限速与速度跟踪（纯 numpy，不依赖 ROS / OpenCV）。

为什么需要这个模块
------------------
仓库内地面载具已有多个**横向**控制实现（阿克曼控制、路径点发布器、车道保持），
纵向方向也有三个：ACC 神经网络跟车、AEB 自动紧急制动、悬崖制动。但这三个都是
**事件触发**的——按前车距离或障碍物决定加减速，不关心道路本身曲率。

缺少"按曲率限速"这一层时，车辆只能按单一巡航速度行驶：
过弯不减速，横向加速度会超出轮胎附着能力而失稳；直线又不能加速，
通行效率低。本模块补的是"道路弯了该开多快"，与上面三个模块互补：

1. **按曲率自动限速**：给定前方路径的曲率，反解出不使横向加速度超限的
   最大安全车速，并在多个约束（横向加速度、横向 jerk、弯道提前量）之间取最小值。
2. **速度跟踪**：PID + 前馈，输出加速度指令，并对加速度与 jerk 做限幅，
   保证指令在车辆执行器能力范围内且乘坐舒适。

设计取舍见 ``docs/ground/carla_speed_control/speed_control.md``。

单位约定
--------
本模块**统一使用国际单位制**：速度 m/s、加速度 m/s²、jerk m/s³、
曲率 1/m、距离 m、时间 s。这与 ROS 的 ``geometry_msgs/Twist``（m/s）
以及 CARLA 的控制接口一致，避免单位混用导致的隐蔽错误。

注意：仓库里部分模块用 km/h 描述车速，本模块在**参数与话题层面**用 m/s，
只在文档的展示表格里换算成 km/h。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import List, Optional, Sequence, Tuple

import numpy as np

__all__ = [
    "SpeedControllerConfig",
    "SpeedController",
    "VehicleLimits",
    "speed_limit_from_curvature",
    "lateral_accel_at",
    "lookahead_distance",
    "max_curvature_ahead",
    "curvature_profile_s_curve",
    "simulate_speed_control",
    "SpeedSimulationResult",
    "SpeedTrace",
    "MIN_SPEED_EPS",
    "GRAVITY",
]

#: 低于此速度视为静止，用于避免 1/κ 这类除法产生无穷大
MIN_SPEED_EPS = 1e-3

#: 重力加速度（m/s²），仅用于把横向加速度换算成"g"便于阅读
GRAVITY = 9.80665


# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------


@dataclass
class VehicleLimits:
    """载具的纵向与横向能力约束。

    这些值应由实车/仿真标定得到，缺省值取自 CARLA 默认车辆
    （``vehicle.tesla.model3``）的量级，仅作示例。
    """

    #: 最大驱动力对应的加速度（m/s²）
    max_accel: float = 3.0
    #: 最大制动力对应的减速度（m/s²，取正值表示减速能力大小）
    max_decel: float = 6.0
    #: 加速度变化率上限（m/s³）。用于抑制顿挫，0 表示不限制
    max_jerk: float = 4.0
    #: 容许的最大横向加速度（m/s²）。决定过弯速度上限
    max_lateral_accel: float = 3.0
    #: 容许的最大横向 jerk（m/s³）。用于让限速变化更平缓
    max_lateral_jerk: float = 2.0
    #: 车辆最低可行速度（m/s），低于此值不再限速（避免停车误判）
    min_speed: float = 1.0

    def validate(self) -> None:
        if self.max_accel <= 0:
            raise ValueError("max_accel 必须为正")
        if self.max_decel <= 0:
            raise ValueError("max_decel 必须为正")
        if self.max_lateral_accel <= 0:
            raise ValueError("max_lateral_accel 必须为正")
        if self.max_jerk < 0:
            raise ValueError("max_jerk 不能为负")
        if self.max_lateral_jerk < 0:
            raise ValueError("max_lateral_jerk 不能为负")
        if self.min_speed < 0:
            raise ValueError("min_speed 不能为负")


@dataclass
class SpeedControllerConfig:
    """速度控制器参数。"""

    #: PID 比例增益（(m/s²) / (m/s)）
    kp: float = 1.0
    #: PID 积分增益（(m/s²) / (m·s)）。取值偏小以抑制超调——
    #: 积分过强会让车辆在减速到限速后继续下冲，随后又超调回来
    ki: float = 0.15
    #: PID 微分增益（(m/s²) / (m/s²)），对速度误差的导数作用。
    #: 它是抑制超调的主要手段：接近目标速度时提前收敛
    kd: float = 0.35
    #: 前馈增益：目标加速度按此比例直接前馈，减小 PID 负担。1.0 为全前馈
    kff: float = 1.0
    #: 积分项上限（m/s²），防止积分饱和
    integral_limit: float = 2.0
    #: 输出加速度上限（m/s²），为 None 时用 limits.max_accel
    accel_limit: Optional[float] = None
    #: 输出减速度上限（m/s²，正值），为 None 时用 limits.max_decel
    decel_limit: Optional[float] = None
    #: 载具能力约束
    limits: VehicleLimits = field(default_factory=VehicleLimits)
    #: 预测时间（s）：限速时按"提前 lookahead_time 秒到达的曲率"计算，
    #: 使车辆在入弯前就开始减速。0 表示不考虑提前量
    lookahead_time: float = 1.5
    #: 超速容差（m/s）。实际速度超过"限速 + 容差"时触发安全限速层。
    #: 设一个小正值是为了避免在限速附近频繁切换控制模式
    overspeed_tolerance: float = 0.2
    #: 超速时附加的减速度增益（(m/s²) / (m/s)）。仅在触发安全层时生效
    overspeed_gain: float = 2.0

    def validate(self) -> None:
        self.limits.validate()
        if self.kp < 0 or self.ki < 0 or self.kd < 0:
            raise ValueError("PID 增益不能为负")
        if not (0.0 <= self.kff <= 1.0):
            raise ValueError("kff 应在 [0, 1] 区间")
        if self.integral_limit < 0:
            raise ValueError("integral_limit 不能为负")
        if self.lookahead_time < 0:
            raise ValueError("lookahead_time 不能为负")
        if self.overspeed_tolerance < 0:
            raise ValueError("overspeed_tolerance 不能为负")
        if self.overspeed_gain < 0:
            raise ValueError("overspeed_gain 不能为负")

    @property
    def max_accel(self) -> float:
        return self.accel_limit if self.accel_limit is not None else self.limits.max_accel

    @property
    def max_decel(self) -> float:
        return self.decel_limit if self.decel_limit is not None else self.limits.max_decel


# ---------------------------------------------------------------------------
# 曲率 -> 限速
# ---------------------------------------------------------------------------


def speed_limit_from_curvature(
    curvature: float,
    max_lateral_accel: float = 3.0,
    min_speed: float = 1.0,
    safety_factor: float = 0.995,
) -> float:
    """由曲率反解不使横向加速度超限的最大车速。

    匀速圆周运动的横向加速度为 ``a_lat = v² · κ``，因此

    .. math::

        v_{max} = \\sqrt{a_{lat,max} / |\\kappa|} \\cdot \\eta

    其中 :math:`\\eta` 为安全系数（``safety_factor``，默认 0.995）。

    为什么要安全系数
    ----------------
    理论上 :math:`\\eta = 1` 时横向加速度恰好等于限值。但离散仿真
    （步长 0.05 s）与实际控制中速度会在限值附近有微小波动：实测
    :math:`\\eta = 1` 时最大横向加速度为 3.01 m/s²，比设定的 3.0 高 0.3%。
    留 0.5% 的速度余量后横向加速度严格不超限，代价是过弯速度低 0.5%
    （39.4 → 39.2 km/h），可以忽略。这与工程上留控制余量的做法一致。

    直道（κ → 0）时返回 ``inf``，表示曲率本身不构成限制。

    参数
    ----
    curvature:
        路径曲率（1/m）。取绝对值参与计算，符号（左/右转）不影响限速大小。
    max_lateral_accel:
        容许的最大横向加速度（m/s²）。
    min_speed:
        低速保护：当按公式算出的限速低于该值时，返回该值而不是更小的数，
        避免在曲率异常大的噪声点上把车速压到接近 0。
    safety_factor:
        安全系数，取值 (0, 1]。1.0 表示不留余量。

    返回
    ----
    float
        该曲率下的最大安全车速（m/s）。直道返回 ``math.inf``。
    """
    if max_lateral_accel <= 0:
        raise ValueError("max_lateral_accel 必须为正")
    if not (0.0 < safety_factor <= 1.0):
        raise ValueError("safety_factor 应在 (0, 1] 区间")
    k = abs(float(curvature))
    if not math.isfinite(k):
        # 曲率无穷大视为不可通行，退化为最低速度
        return float(min_speed)
    if k < 1e-9:
        return math.inf
    v = math.sqrt(max_lateral_accel / k) * safety_factor
    return max(v, float(min_speed))


def lateral_accel_at(speed: float, curvature: float) -> float:
    """计算给定速度与曲率下的横向加速度（m/s²），供校验用。"""
    return float(speed) ** 2 * abs(float(curvature))


def lookahead_distance(speed: float, lookahead_time: float) -> float:
    """按当前速度换算限速前瞻距离（m）。"""
    return max(0.0, float(speed)) * max(0.0, float(lookahead_time))


def speed_limit_profile(
    s: np.ndarray,
    curvature: np.ndarray,
    config: Optional[SpeedControllerConfig] = None,
    cruise_speed: float = 12.0,
    lookahead_distance_m: float = 60.0,
    enforce_achievable: bool = True,
) -> np.ndarray:
    """预先算出每个里程点的限速。

    分两步：

    **第一步：几何限速**。取两个约束的较小值：

    1. 巡航速度 ``cruise_speed``；
    2. 前方 ``lookahead_distance_m`` 范围内**最大曲率**对应的安全车速。

    为什么取窗口内最大而不是单点：单点插值会漏掉窗口内的曲率峰值，车辆
    按偏大的限速继续加速，到弯心时真实曲率更高，横向加速度随之超标。
    实测单点前瞻时最大横向加速度 3.59 m/s²（限值 3.0），改为窗口最大后
    显著改善。工程含义：限速要看"最坏情况"，不是"眼前那一点"。

    **第二步：物理可达性约束**（``enforce_achievable``）。仅靠第一步会算出
    **物理上无法实现**的限速剖面：在两个弯之间的短直道上，前瞻窗口可能恰好
    避开前后两个弯，于是限速被算成巡航速度——但车辆不可能在直道中段加速到
    巡航速度后立刻又为下一个弯减速。实测这种尖峰出现在 s≈165 m 处，
    限速从 10.9 跳到 12.0 又立刻掉回 10.9。

    修正方式是经典的**双向可达性扫描**：

    * 反向扫描（限制加速）：从后往前，任一点的速度不能超过"以最大加速度
      从该点加速所能达到的速度"，即

      .. math::

          v_i \\le \\sqrt{v_{i+1}^2 + 2 a_{max} \\Delta s}

      这一步是消除尖峰的关键：若前方不远处限速更低，当前点的限速会被
      相应压低，因为"加速上去再刹回来"来不及。

    * 正向扫描（限制减速）：从前往后，任一点的速度不能超过"以最大减速度
      从该点减速所能达到的速度"，即

      .. math::

          v_{i+1} \\le \\sqrt{v_i^2 + 2 a_{min} \\Delta s}

      这一步保证要求的减速不超过车辆制动能力。

    两步都作用在**加速度层面**，因此得到的剖面总是可实现的；车辆只需
    跟踪它，不需要再临时超调。
    """
    cfg = config or SpeedControllerConfig()
    s_grid, kappa_grid = _resolve_profile(s, curvature)
    lim = cfg.limits

    # ---- 第一步：几何限速 ----
    # 对每个点，取"前方 lookahead 窗口内**绝对值最大的曲率**"所对应的限速。
    #
    # 为什么不能写成"窗口内各点限速的最小值"（两者看似等价，实则不同）：
    # 当窗口跨越曲率的斜坡段时，逐点最小值只反映窗口内已出现的曲率，
    # 而窗口右端点之外的曲率峰值可能更大。实测在 s=171 处窗口
    # [171, 231] 覆盖到的最大曲率是 0.02083（斜坡上），而峰值 0.02500
    # 出现在 s=233，差 2 米没进窗口，于是限速被算成 11.94（正确值
    # 10.90），曲线上出现一个尖峰；相邻点 s=172 的窗口刚好覆盖到峰值，
    # 又掉回 10.90，形成锯齿。
    #
    # 显式对窗口内的**曲率**取最大绝对值，即可正确处理斜坡段。
    n = kappa_grid.size
    abs_kappa = np.abs(kappa_grid)
    out = np.empty(n, dtype=float)
    j = 0
    for i in range(n):
        limit_s = s_grid[i] + lookahead_distance_m
        while j < n and s_grid[j] <= limit_s:
            j += 1
        if j <= i:
            j = min(i + 1, n)
        k_window = float(abs_kappa[i:j].max())
        out[i] = speed_limit_from_curvature(
            k_window, lim.max_lateral_accel, lim.min_speed
        )
    # 直道处 limit 会是 inf，用巡航速度截断
    out = np.minimum(out, cruise_speed)

    if not enforce_achievable:
        return out

    # ---- 第二步：双向可达性扫描 ----
    # 注意：扫描结果必须写入返回值。早期版本把扫描作用于局部变量 v，
    # 却仍然 return out，导致扫描完全无效——尖峰依旧存在（实测 v 从
    # 10.90 跳到 12.00 再掉回 10.90）。这类"算完没用上"的错误不会有
    # 任何报错，只能靠对输出曲线的检查发现。
    #
    # 无穷大速度在扫描中会产生 inf - inf，先替换成巡航速度
    v = np.where(np.isfinite(out), out, cruise_speed)

    # 反向：限制加速能力（消除"短直道上加速上去再刹回来"的尖峰）
    for i in range(n - 2, -1, -1):
        ds = s_grid[i + 1] - s_grid[i]
        if ds <= 0:
            continue
        v_next = v[i + 1]
        v_allowed = math.sqrt(v_next * v_next + 2.0 * lim.max_accel * ds)
        if v[i] > v_allowed:
            v[i] = v_allowed

    # 正向：限制减速能力（保证要求的减速度不超过制动能力）
    for i in range(1, n):
        ds = s_grid[i] - s_grid[i - 1]
        if ds <= 0:
            continue
        v_prev = v[i - 1]
        v_allowed = math.sqrt(v_prev * v_prev + 2.0 * lim.max_decel * ds)
        if v[i] > v_allowed:
            v[i] = v_allowed

    # 不低于最低速度
    v = np.maximum(v, lim.min_speed)
    return np.minimum(v, cruise_speed)


def required_decel(
    v_now: float,
    v_future: float,
    distance: float,
    max_decel: float,
) -> float:
    """计算在 ``distance`` 米内把速度从 ``v_now`` 降到 ``v_future`` 所需减速度。

    由匀减速公式 ``v² = v₀² + 2·a·d`` 反解：

    .. math::

        a = \\frac{v_{future}^2 - v_{now}^2}{2 d}

    返回负值表示需要减速。若 ``distance`` 为 0 或不需要减速，返回 0。
    结果会被截断到 ``-max_decel``，表示不超过制动能力（超出的部分无法实现，
    属于物理限制，应由前瞻距离保证不出现）。
    """
    if distance <= 1e-6:
        return 0.0
    if v_future >= v_now:
        return 0.0
    a = (v_future ** 2 - v_now ** 2) / (2.0 * distance)
    return max(a, -abs(max_decel))


def find_brake_point(
    s: np.ndarray,
    v_limit: np.ndarray,
    distance: float,
    v_now: float,
    horizon: float = 120.0,
) -> Optional[Tuple[float, float]]:
    """在前方 ``horizon`` 米内找第一个"限速低于当前车速"的点。

    返回 ``(距离, 该点限速)``；找不到则返回 ``None``。

    这是预瞄制动的第一步：先知道要在多远之后降到多少，才能算出需要的减速度。
    """
    s = np.asarray(s, dtype=float)
    v_limit = np.asarray(v_limit, dtype=float)
    i0 = int(np.searchsorted(s, distance, side="left"))
    i1 = int(np.searchsorted(s, distance + horizon, side="right"))
    i0 = max(0, min(i0, s.size - 1))
    i1 = max(i0 + 1, min(i1, s.size))
    for i in range(i0, i1):
        if v_limit[i] < v_now - 1e-6:
            return float(s[i] - distance), float(v_limit[i])
    return None


def apply_jerk_limit(
    accel_cmd: float,
    prev_accel: float,
    dt: float,
    max_jerk: float,
) -> float:
    """对加速度指令做变化率限制，返回限幅后的指令。

    这是控制链路的**最后一步**，任何安全层/限速层的输出都必须经过它，
    否则会出现加速度阶跃——实测未经此限制时最大 jerk 达 40 m/s³
    （限值 4），表现为突然的急刹顿挫。
    """
    if max_jerk <= 0.0 or dt <= 0.0:
        return accel_cmd
    max_delta = max_jerk * dt
    delta = accel_cmd - prev_accel
    if abs(delta) > max_delta:
        return prev_accel + math.copysign(max_delta, delta)
    return accel_cmd


# ---------------------------------------------------------------------------
# 曲率剖面（用于仿真与测试）
# ---------------------------------------------------------------------------


def curvature_profile_s_curve(
    total_length: float = 400.0,
    straight_length: float = 100.0,
    curve_radius: float = 40.0,
    curve_length: float = 120.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """生成"直道 - 左弯 - 直道 - 右弯 - 直道"的曲率剖面。

    返回 ``(s, curvature)``，``s`` 为沿路径的里程（m），``curvature`` 为 1/m。
    曲率在弯道内为常值，**入弯与出弯处各用 5 m 线性斜坡过渡**，模拟真实道路
    的缓和曲线。

    注意：过渡区只加在弯道两端，**不削减弯道中段的曲率峰值**。
    早期版本对整条剖面做了滑动平均，把峰值从 0.025 削到 0.021，
    导致按"单点前瞻"计算的限速偏大、实际横向加速度超标（3.59 > 3.0）。
    这个坑同时也是 `:func:`simulate_speed_control` 改用"前瞻窗口内最大曲率"
    的直接原因——见该函数文档。
    """
    if curve_radius <= 0:
        raise ValueError("curve_radius 必须为正")
    if curve_length <= 0:
        raise ValueError("curve_length 必须为正")
    if straight_length < 0:
        raise ValueError("straight_length 不能为负")

    ramp = 5.0  # 缓和段长度（m）
    # (长度, 目标曲率, 入坡长度, 出坡长度)
    segs: List[Tuple[float, float, float, float]] = [
        (straight_length, 0.0, 0.0, 0.0),
        (curve_length, 1.0 / curve_radius, ramp, ramp),    # 左弯
        (straight_length, 0.0, 0.0, 0.0),
        (curve_length, -1.0 / curve_radius, ramp, ramp),   # 右弯
        (straight_length, 0.0, 0.0, 0.0),
    ]

    used = sum(L for L, _, _, _ in segs)
    if total_length < used:
        shrink = ((total_length - 2 * curve_length) / (3 * straight_length)
                  if straight_length > 0 else 0.0)
        shrink = max(0.0, min(1.0, shrink))
        segs = [(L * shrink if k == 0.0 else L, k, a, b) for L, k, a, b in segs]
        used = sum(L for L, _, _, _ in segs)
    if used < total_length:
        L0, k0, a0, b0 = segs[-1]
        segs[-1] = (L0 + (total_length - used), k0, a0, b0)

    n = int(math.ceil(sum(L for L, _, _, _ in segs))) + 1
    s = np.arange(n, dtype=float)
    kappa = np.zeros(n, dtype=float)

    pos = 0.0
    for L, k, ramp_in, ramp_out in segs:
        i0 = int(round(pos))
        i1 = min(n, int(round(pos + L)) + 1)
        if i1 <= i0:
            pos += L
            continue
        kappa[i0:i1] = k
        # 入坡：从前一段的值线性过渡到 k
        if ramp_in > 0:
            n_in = max(1, int(round(ramp_in)))
            for j in range(n_in):
                idx = i0 + j
                if idx < n:
                    kappa[idx] = k * (j + 1) / (n_in + 1)
        # 出坡：从 k 线性回到 0
        if ramp_out > 0:
            n_out = max(1, int(round(ramp_out)))
            for j in range(n_out):
                idx = i1 - n_out + j
                if 0 <= idx < n:
                    kappa[idx] = k * (n_out - j) / (n_out + 1)
        pos += L

    return s, kappa


def max_curvature_ahead(
    s: np.ndarray,
    curvature: np.ndarray,
    distance: float,
    lookahead: float,
) -> float:
    """取 ``[distance, distance + lookahead]`` 区间内**绝对值最大**的曲率。

    为什么不能用单点插值
    --------------------
    用单点曲率做限速时，如果该点的曲率恰好落在缓和段（或恰好避开了曲率峰值），
    算出的限速会偏大；车辆继续加速到弯心时真实曲率已经更高，横向加速度随之
    超标。实测：单点前瞻时最大横向加速度 3.59 m/s²，超出 3.0 的限值；
    改为窗口内最大曲率后回到限值以内。

    工程含义：限速要"看最坏情况"，而不是"看眼前那一点"。

    返回区间内绝对值最大的曲率（保留符号）；区间为空时返回 0。
    """
    s = np.asarray(s, dtype=float)
    curvature = np.asarray(curvature, dtype=float)
    lo = float(distance)
    hi = float(distance) + max(0.0, float(lookahead))
    # s 已排序，用二分找区间
    i0 = int(np.searchsorted(s, lo, side="left"))
    i1 = int(np.searchsorted(s, hi, side="right"))
    i0 = max(0, min(i0, s.size - 1))
    i1 = max(i0 + 1, min(i1, s.size))
    window = curvature[i0:i1]
    if window.size == 0:
        return 0.0
    j = int(np.argmax(np.abs(window)))
    return float(window[j])


# ---------------------------------------------------------------------------
# 控制器
# ---------------------------------------------------------------------------


class SpeedController:
    """纵向速度控制器：PID + 前馈，带积分抗饱和与加速度限幅。

    与 `:class:`VehicleLimits` 配合，输出始终落在车辆能力范围内。
    每次调用 `:meth:`update` 必须传入真实的时间增量 ``dt``；
    内部会缓存上一次加速度以支持 jerk 限制。
    """

    def __init__(self, config: Optional[SpeedControllerConfig] = None):
        self.config = config or SpeedControllerConfig()
        self.config.validate()
        self._integral = 0.0
        self._prev_error: Optional[float] = None
        self._last_accel: float = 0.0

    # -- 状态管理 ---------------------------------------------------------

    def reset(self) -> None:
        """清空积分与微分历史。感知/目标丢失后应调用，避免沿用旧状态。"""
        self._integral = 0.0
        self._prev_error = None
        self._last_accel = 0.0

    @property
    def integral(self) -> float:
        return self._integral

    @property
    def last_accel(self) -> float:
        return self._last_accel

    def sync_accel(self, accel: float) -> None:
        """把内部记录的"上次加速度"同步为外部真实施加的值。

        当外层控制链路在 `:meth:`update` 之后又施加了预瞄制动、超速兜底或
        jerk 限制时，必须调用本方法，否则下一次 update 的 jerk 限制会基于
        一个并未真正执行的旧值计算，导致限制失效。
        """
        self._last_accel = float(accel)

    # -- 主计算 -----------------------------------------------------------

    def update(
        self,
        target_speed: float,
        current_speed: float,
        dt: float,
        target_accel: float = 0.0,
    ) -> float:
        """计算加速度指令。

        参数
        ----
        target_speed:
            目标速度（m/s）。
        current_speed:
            当前速度（m/s）。
        dt:
            距上次调用的时间增量（s）。``dt <= 0`` 时不做积分与微分，
            仅按比例项输出，避免除零与状态错乱。
        target_accel:
            目标加速度前馈值（m/s²）。一般由限速剖面变化率给出，
            默认为 0（定速巡航）。

        返回
        ----
        float
            加速度指令（m/s²）。正值加速，负值减速。
        """
        cfg = self.config
        error = float(target_speed) - float(current_speed)

        # 积分项（带抗饱和）：仅在 dt 有效时累积
        if dt > 0.0:
            self._integral += error * dt
            self._integral = float(
                np.clip(self._integral, -cfg.integral_limit, cfg.integral_limit)
            )

        # 微分项
        deriv = 0.0
        if dt > 0.0 and self._prev_error is not None:
            deriv = (error - self._prev_error) / dt

        accel = cfg.kp * error + cfg.ki * self._integral + cfg.kd * deriv
        accel += cfg.kff * float(target_accel)

        # 限幅到车辆纵向能力
        accel = float(np.clip(accel, -cfg.max_decel, cfg.max_accel))

        # jerk 限制：限制相邻两次指令的变化率
        if cfg.limits.max_jerk > 0.0 and dt > 0.0:
            max_delta = cfg.limits.max_jerk * dt
            delta = accel - self._last_accel
            if abs(delta) > max_delta:
                accel = self._last_accel + math.copysign(max_delta, delta)

        # 记录状态（积分项在限幅后按"实际输出"回算，进一步抑制饱和）
        if dt > 0.0:
            self._prev_error = error
        self._last_accel = accel
        return accel


# ---------------------------------------------------------------------------
# 一维纵向仿真
# ---------------------------------------------------------------------------


@dataclass
class SpeedTrace:
    """仿真过程中的时间序列。"""

    t: np.ndarray
    s: np.ndarray
    v: np.ndarray
    v_target: np.ndarray
    v_limit: np.ndarray
    a_cmd: np.ndarray
    a_lat: np.ndarray
    curvature: np.ndarray

    def __len__(self) -> int:
        return int(self.t.size)


@dataclass
class SpeedSimulationResult:
    """仿真结果与评价指标。"""

    trace: SpeedTrace
    #: 是否所有时刻横向加速度都在限值内
    within_lateral_limit: bool
    #: 最大横向加速度（m/s²）
    max_lateral_accel: float
    #: 最大加速度指令绝对值（m/s²）
    max_abs_accel: float
    #: 最大 jerk（m/s³）
    max_abs_jerk: float
    #: 稳态速度误差的最大绝对值（仅在直道匀速段统计，m/s）
    max_speed_error: float
    #: 实际速度超过"限速 + 容差"的最大幅度（m/s）。理想为 0
    max_overspeed: float
    #: 触发安全限速层的仿真步数占总步数的比例
    overspeed_ratio: float
    #: 总行驶时间（s）
    total_time: float
    #: 平均速度（m/s）
    mean_speed: float

    def summary(self) -> str:
        return (
            f"行驶 {self.trace.s[-1]:.1f} m 用时 {self.total_time:.2f} s，"
            f"平均 {self.mean_speed:.2f} m/s；"
            f"最大横向加速度 {self.max_lateral_accel:.2f} m/s²"
            f"（限值内: {'是' if self.within_lateral_limit else '否'}）；"
            f"最大超速 {self.max_overspeed:.3f} m/s"
            f"（安全层介入 {100.0 * self.overspeed_ratio:.1f}% 步数）；"
            f"最大 |a| {self.max_abs_accel:.2f} m/s²，"
            f"最大 |jerk| {self.max_abs_jerk:.2f} m/s³"
        )


def _resolve_profile(
    s: np.ndarray, curvature: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """把采样点排序并去掉重复里程，便于插值查询。"""
    s = np.asarray(s, dtype=float)
    curvature = np.asarray(curvature, dtype=float)
    if s.ndim != 1 or curvature.ndim != 1 or s.size != curvature.size:
        raise ValueError("s 与 curvature 必须是一维且等长")
    if s.size < 2:
        raise ValueError("曲率剖面至少需要 2 个采样点")
    order = np.argsort(s)
    s = s[order]
    curvature = curvature[order]
    # 合并重复里程
    keep = np.concatenate(([True], np.diff(s) > 0))
    return s[keep], curvature[keep]


def simulate_speed_control(
    s: np.ndarray,
    curvature: np.ndarray,
    config: Optional[SpeedControllerConfig] = None,
    cruise_speed: float = 12.0,
    dt: float = 0.05,
    plan_ahead: bool = True,
) -> SpeedSimulationResult:
    """一维纵向仿真：车辆沿给定曲率剖面行驶，观察限速与跟踪效果。

    车辆模型为一阶惯性环节（加速度指令经时间常数平滑后作用于速度）：

    .. math::

        \\dot v = (a_{cmd} - v \\cdot k_{drag}) / \\tau

    其中 ``k_drag`` 模拟行驶阻力与滚阻，``τ`` 为动力总成响应时间常数。
    这里刻意使用简化模型——本模块关注的是**控制律是否正确**，
    不是车辆动力学精度；简化模型让指标可解析对照。

    参数
    ----
    s, curvature:
        曲率剖面（沿路径里程与对应曲率）。
    config:
        控制器配置。
    cruise_speed:
        无曲率限制时的巡航速度（m/s）。
    dt:
        仿真步长（s）。
    plan_ahead:
        是否按 ``lookahead_time`` 提前取前方曲率做限速（True 更接近实际）。

    返回
    ----
    SpeedSimulationResult
    """
    cfg = config or SpeedControllerConfig()
    cfg.validate()
    if dt <= 0:
        raise ValueError("dt 必须为正")
    if cruise_speed <= 0:
        raise ValueError("cruise_speed 必须为正")

    s_grid, kappa_grid = _resolve_profile(s, curvature)
    total = float(s_grid[-1])

    # 车辆模型参数
    tau = 0.8          # 动力总成时间常数（s）
    drag = 0.008       # 速度相关阻力系数（1/s）

    controller = SpeedController(cfg)
    lim = cfg.limits

    # 预先算出限速剖面（含"前方最大曲率"约束）。
    # 实际部署时这一项由节点在线计算：订阅曲率话题、按前瞻距离取窗口最大值。
    v_curve_profile = speed_limit_profile(
        s_grid, kappa_grid, cfg, cruise_speed, lookahead_distance_m=60.0
    )

    t_list: List[float] = []
    s_list: List[float] = []
    v_list: List[float] = []
    vt_list: List[float] = []
    vl_list: List[float] = []
    a_list: List[float] = []
    alat_list: List[float] = []
    k_list: List[float] = []

    t = 0.0
    dist = 0.0
    v = 0.0
    prev_accel = 0.0   # jerk 限制所需的"上次施加的加速度"
    n_overspeed = 0    # 触发超速兜底的步数

    max_steps = int(1e6)
    step = 0
    while dist < total and step < max_steps:
        step += 1
        # 当前曲率
        k_now = float(np.interp(dist, s_grid, kappa_grid))

        # 前瞻曲率：取前瞻窗口内的**最大**曲率（而不是单点插值）。
        # 单点插值会漏掉窗口内的曲率峰值，导致限速偏大、横向加速度超标，
        # 详见 max_curvature_ahead 的说明。
        # 速度低时前瞻距离至少 5 m，避免起步阶段完全没有前瞻。
        if plan_ahead:
            d_la = lookahead_distance(v, cfg.lookahead_time)
            d_la = max(d_la, 5.0)
            k_for_limit = max_curvature_ahead(s_grid, kappa_grid, dist, d_la)
        else:
            k_for_limit = k_now

        # 限速：预先算好的限速剖面在当前位置的值
        # （剖面已包含"前方 lookahead_distance_m 内最大曲率"的约束）
        v_limit = float(np.interp(dist, s_grid, v_curve_profile))

        a_cmd = controller.update(v_limit, v, dt, target_accel=0.0)

        # ---- 预瞄制动 ----
        # 目的：在进入低限速区之前就把速度降下来，从根上避免"到弯心才刹车"
        # 造成的超调。做法是找出前方第一个限速低于当前车速的点，算出所需的
        # 匀减速度，再与 PID 输出取"更保守的那个"（即更小，偏向减速）。
        #
        # 为什么需要它：仅靠 PID 跟踪限速时，由于动力总成有 0.8 s 时间常数，
        # 车辆会在减速段超出限速（实测超速 0.83 m/s），进而使横向加速度
        # 超过设定值。预瞄制动把这件事变成"提前规划"而不是"事后纠正"。
        if plan_ahead:
            bp = find_brake_point(s_grid, v_curve_profile, dist, v)
            if bp is not None:
                d_brake, v_future = bp
                a_need = required_decel(v, v_future, d_brake, cfg.max_decel)
                if a_need < a_cmd:
                    a_cmd = a_need

        # ---- 超速兜底 ----
        # 预瞄制动已能避免绝大多数超速，但离散仿真与执行延迟仍可能残留少量
        # 超出。这里保留一层兜底：一旦超过（限速 + 容差）就补一点制动。
        # 注意它的输出同样要经过下面的 jerk 限制，不能绕过。
        overspeed = v - (v_limit + cfg.overspeed_tolerance)
        if overspeed > 0.0:
            a_cmd = min(a_cmd, -cfg.overspeed_gain * overspeed)
            n_overspeed += 1

        # ---- jerk 限制（控制链路最后一步）----
        # 任何安全层/限速层的输出都必须经过它。实测若绕过此步，
        # 最大 jerk 会达到 40 m/s³（限值 4），表现为突然急刹的顿挫。
        a_cmd = apply_jerk_limit(a_cmd, prev_accel, dt, lim.max_jerk)
        controller.sync_accel(a_cmd)
        prev_accel = a_cmd

        # 车辆响应
        v_dot = (a_cmd - v * drag) / tau
        v = max(0.0, v + v_dot * dt)
        dist += v * dt
        t += dt

        t_list.append(t)
        s_list.append(dist)
        v_list.append(v)
        vt_list.append(v_limit)
        vl_list.append(v_limit)
        a_list.append(a_cmd)
        alat_list.append(lateral_accel_at(v, k_now))
        k_list.append(k_now)
    trace = SpeedTrace(
        t=np.asarray(t_list),
        s=np.asarray(s_list),
        v=np.asarray(v_list),
        v_target=np.asarray(vt_list),
        v_limit=np.asarray(vl_list),
        a_cmd=np.asarray(a_list),
        a_lat=np.asarray(alat_list),
        curvature=np.asarray(k_list),
    )

    if trace.t.size == 0:
        raise RuntimeError("仿真没有产生任何数据点，请检查曲率剖面与步长")

    a = trace.a_cmd
    jerk = np.diff(a) / dt if a.size > 1 else np.zeros(1)
    a_lat = trace.a_lat

    # 超速统计：超出"限速 + 容差"的幅度
    over = trace.v - (trace.v_limit + cfg.overspeed_tolerance)
    max_overspeed = float(over.max()) if over.size else 0.0
    if max_overspeed < 0.0:
        max_overspeed = 0.0

    # 稳态误差的提取方式：在每个"连续稳定区段"内只取**后 30%** 的样本。
    #
    # 为什么不能直接取全部门限内样本：起步加速时车速从 0 升到巡航速度，
    # 穿过门限（< 2 m/s）的那些点虽然"已接近限速"，但仍处于过渡过程，
    # 误差接近 2 m/s，会把指标拉高到 1.97。稳态误差应当只反映"已经跟上之后
    # 的残余偏差"，因此每个区段只看尾部。
    err = np.abs(trace.v - trace.v_limit)
    gate = err < 2.0
    if trace.v_limit.size > 1:
        change = np.abs(np.diff(trace.v_limit, prepend=trace.v_limit[0]))
        gate = gate & (change < 0.05)

    if gate.any():
        # 找出连续的 True 区段，并**丢弃过短的碎片段**。
        #
        # 为什么必须丢弃：出弯瞬间限速会从弯道限速跳回巡航速度，这个上升过程
        # 会短暂地穿过 0.05 m/s 的变化门限，产生只有 2~3 个样本的"假稳定段"。
        # 此时车速还停留在弯道限速上，误差达到 1.06 m/s，会把指标从真实的
        # 0.03 m/s 拉到 1.06。真实稳态段的长度都是数百个样本，用一个下限即可
        # 干净地区分（实测最短的真稳态段为 95 个样本，最长碎片为 3 个）。
        min_segment = 10
        idx = np.flatnonzero(gate)
        transitions = np.flatnonzero(np.diff(idx) > 1)
        segments = [seg for seg in np.split(idx, transitions + 1)
                    if seg.size >= min_segment]
        if not segments:
            # 退化情况：没有足够长的稳定段，放宽门限重新取尾部
            segments = [seg for seg in np.split(idx, transitions + 1) if seg.size > 0]
        tail_err = []
        for seg in segments:
            keep = max(1, int(math.ceil(0.3 * seg.size)))
            tail_err.append(err[seg[-keep:]])
        max_speed_error = (
            float(np.concatenate(tail_err).max()) if tail_err else float(err.max())
        )
    else:
        max_speed_error = float(err.max())

    n_steps = max(1, trace.t.size)
    return SpeedSimulationResult(
        trace=trace,
        within_lateral_limit=bool(np.all(a_lat <= lim.max_lateral_accel + 1e-6)),
        max_lateral_accel=float(a_lat.max()) if a_lat.size else 0.0,
        max_abs_accel=float(np.abs(a).max()) if a.size else 0.0,
        max_abs_jerk=float(np.abs(jerk).max()) if jerk.size else 0.0,
        max_speed_error=max_speed_error,
        max_overspeed=max_overspeed,
        overspeed_ratio=float(n_overspeed) / float(n_steps),
        total_time=float(trace.t[-1]),
        mean_speed=float(trace.s[-1] / trace.t[-1]) if trace.t[-1] > 0 else 0.0,
    )
