#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三种横向控制器：纯跟踪、Stanley、LQR。

三者接口完全一致::

    command = controller.compute(state, speed, path, projection, dt)

返回 ``[前轮转角 delta, 诊断信息 dict]``。纵向速度由统一的 ``speed_control``
模块给出，与横向控制器解耦——这样对比只反映"横向算法"的差异，不掺杂
速度策略的影响，是本模块评测公平性的关键设计。

符号约定（三种控制器一致）：
* ``delta > 0`` 为左转（逆时针），横摆角速度为正；
* 横向偏差 ``e`` 以参考路径左法向为正（左偏为正）；
* 航向偏差 ``psi_e = wrap(yaw - path_yaw)``（车头相对路径，左偏为正）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np

from .geometry import Projection, ReferencePath, wrap_to_pi
from .vehicle_model import VehicleParams


@dataclass
class ControllerResult:
    """控制器的输出。"""

    delta: float                      # 前轮转角 (rad)
    info: Dict[str, float] = field(default_factory=dict)


class BaseController:
    """控制器基类：统一的转角限幅与转向速率限制。"""

    name = "base"

    def __init__(self, params: VehicleParams | None = None,
                 steering_rate_limit: float = math.radians(150.0)) -> None:
        self.p = params or VehicleParams()
        self.steering_rate_limit = float(steering_rate_limit)
        self._delta = 0.0

    def reset(self) -> None:
        self._delta = 0.0

    def _limit(self, delta: float, dt: float) -> float:
        delta = float(np.clip(delta, -self.p.max_steering, self.p.max_steering))
        max_step = self.steering_rate_limit * dt
        delta = float(np.clip(delta, self._delta - max_step, self._delta + max_step))
        self._delta = delta
        return delta

    def compute(self, state: np.ndarray, speed: float, path: ReferencePath,
                projection: Projection, dt: float) -> ControllerResult:
        raise NotImplementedError


class PurePursuit(BaseController):
    """纯跟踪：以自适应前视距离处的路径点为跟踪目标。

    前视距离随车速线性增长 ``Ld = clamp(k_ld * v, Ld_min, Ld_max)``，
    转角由几何关系给出::

        delta = atan( 2 * L * sin(alpha) / Ld )

    ``alpha`` 为车头朝向与前视点方向之间的夹角。
    """

    name = "pure_pursuit"

    def __init__(self, lookahead_gain: float = 0.6, lookahead_min: float = 3.0,
                 lookahead_max: float = 12.0, params: VehicleParams | None = None,
                 steering_rate_limit: float = math.radians(150.0)) -> None:
        super().__init__(params, steering_rate_limit)
        self.lookahead_gain = float(lookahead_gain)
        self.lookahead_min = float(lookahead_min)
        self.lookahead_max = float(lookahead_max)

    def lookahead(self, speed: float) -> float:
        return float(np.clip(self.lookahead_gain * max(speed, 0.0),
                             self.lookahead_min, self.lookahead_max))

    def compute(self, state, speed, path, projection, dt) -> ControllerResult:
        x, y, yaw, _ = state
        ld = self.lookahead(speed)
        target = path.point_at_s(projection.s + ld)
        alpha = float(wrap_to_pi(math.atan2(target[1] - y, target[0] - x) - yaw))
        distance = max(float(np.hypot(target[0] - x, target[1] - y)), 1e-6)
        delta = math.atan(2.0 * self.p.wheelbase * math.sin(alpha) / distance)
        delta = self._limit(delta, dt)
        return ControllerResult(delta, {"lookahead": ld, "alpha": alpha})

    def reset(self) -> None:
        super().reset()


class Stanley(BaseController):
    """Stanley：横向偏差 + 航向偏差 + 曲率前馈。

    ::

        delta = psi_e - atan(k * e / (k_soft + v)) + atan(L * kappa)

    其中 ``e`` 的取点由 ``use_rear_axle`` 决定：

    * ``False``（默认，**经典写法**）：取**前轴中心**的横向偏差。前轴比车辆
      实际位置超前一个轴距，在定曲率弯道上会表现出一个稳定的内偏/外偏，
      实测半径 20 m 圆弧上稳态偏差可达 0.5 m 量级。
    * ``True``：改在**后轴中心**（即车辆状态点）取偏差，与运动学模型的参考点
      一致，同一场景稳态偏差可降到 0.01 m 量级。

    这是本项目的一个实测结论，也是本模块的默认选择：**取点位置**这类实现细节
    对性能的影响，可能比算法本身更大。用 ``--mode sweep`` 可以复现这组对照。

    注意 ``psi_e`` 用 ``path_yaw - yaw``、横向偏差项取负号；写反会让闭环变成
    正反馈（这是这类实现最常见的错误）。
    """

    name = "stanley"

    def __init__(self, k: float = 4.0, k_soft: float = 1.0,
                 use_feedforward: bool = True, use_rear_axle: bool = True,
                 params: VehicleParams | None = None,
                 steering_rate_limit: float = math.radians(150.0)) -> None:
        super().__init__(params, steering_rate_limit)
        self.k = float(k)
        self.k_soft = float(k_soft)
        self.use_feedforward = bool(use_feedforward)
        self.use_rear_axle = bool(use_rear_axle)

    def compute(self, state, speed, path, projection, dt) -> ControllerResult:
        x, y, yaw, _ = state
        if self.use_rear_axle:
            # 后轴参考：车辆状态点本身就是控制点
            ctrl_proj = projection
            e = projection.signed_distance
        else:
            # 前轴参考（经典 Stanley）
            fx = x + self.p.wheelbase * math.cos(yaw)
            fy = y + self.p.wheelbase * math.sin(yaw)
            ctrl_proj = path.project(np.array([fx, fy]))
            e = ctrl_proj.signed_distance
        path_yaw = math.atan2(ctrl_proj.tangent[1], ctrl_proj.tangent[0])
        psi_e = float(wrap_to_pi(path_yaw - yaw))
        feedback = -math.atan(self.k * e / (self.k_soft + max(speed, 0.0)))
        ff = 0.0
        if self.use_feedforward:
            ff = math.atan(self.p.wheelbase * path.curvature_at_s(ctrl_proj.s))
        delta = self._limit(psi_e + feedback + ff, dt)
        return ControllerResult(delta, {"e": e, "psi_e": psi_e, "feedforward": ff})


class LQR(BaseController):
    """LQRI：误差动力学 + 曲率前馈 + 横向偏差积分的横向控制器。

    误差状态 ``e = [横向偏差, 航向偏差, 横摆角速度]``，控制量为**期望横摆角速度**。

    推导（小角度、左转为正，与全文符号约定一致）::

        e'   = v * psi
        psi' = r - kappa * v
        r'   = (r_des - r) / T           # 执行器一阶滞后，T 为时间常数

    线性化后::

        A = [[0, v, 0], [0, 0, 1], [0, 0, -1/T]]
        B = [[0], [0], [1/T]]

    曲率前馈 ``r_ref = kappa * v``，控制律::

        r_des = r_ref - K·e - Ki·z,      z' = e

    **为什么必须有积分项**：只有比例状态反馈时，起步瞬态（低速加速 + 执行器饱和）
    造成的偏差不会被消掉，而是被锁在一个恒定值——实测直道残留约 0.05 m、
    圆弧上会累积到十几米。加入横向偏差积分后稳态误差趋零。

    **为什么不加预瞄**：LQR 的误差状态里已含航向偏差，再对预瞄点取误差会把
    航向信息重复计入，实测反而使误差累积（预瞄 5 m 时圆弧误差从 0.03 m 升到 14 m）。
    因此默认 ``lookahead=0``，与纯跟踪（必须前视）和 Stanley（按前轴取点）形成对照。

    符号：车辆在路径左侧（``e>0``）需右转，反馈取负号；写反会立即发散。
    """

    name = "lqr"

    def __init__(self, q_diag: Tuple[float, float, float] = (20.0, 2.0, 0.5),
                 r: float = 1.0, lookahead: float = 0.0,
                 ki: float = 0.8, integral_limit: float = 2.0,                 params: VehicleParams | None = None,
                 steering_rate_limit: float = math.radians(150.0),
                 response_time: float = 0.12) -> None:
        super().__init__(params, steering_rate_limit)
        self.q_diag = np.diag(np.asarray(q_diag, dtype=float))
        self.r = float(r)
        self.lookahead = float(lookahead)
        self.ki = float(ki)
        self.integral_limit = float(integral_limit)
        self.response_time = float(response_time)
        self._gain_cache: Dict[float, np.ndarray] = {}
        self._r = 0.0
        self._integral = 0.0

    def reset(self) -> None:
        super().reset()
        self._r = 0.0
        self._integral = 0.0

    def gain(self, speed: float) -> np.ndarray:
        """按车速解一次 Riccati 方程（按速度缓存）。"""
        v = max(float(speed), 0.5)
        key = round(v, 2)
        if key in self._gain_cache:
            return self._gain_cache[key]
        from .lqr_solver import solve_care
        t = self.response_time
        a = np.array([[0.0, v, 0.0],
                      [0.0, 0.0, 1.0],
                      [0.0, 0.0, -1.0 / t]])
        b = np.array([[0.0], [0.0], [1.0 / t]])
        try:
            p = solve_care(a, b, self.q_diag, np.array([[self.r]]))
            gain = (b.T @ p)[0] / self.r
        except Exception:
            gain = np.array([0.3, 0.9, 0.4])
        self._gain_cache[key] = gain
        return gain

    def compute(self, state, speed, path, projection, dt) -> ControllerResult:
        x, y, yaw, v = state
        v_eff = max(float(v), 0.5)
        l = self.p.wheelbase
        t = self.response_time

        # 误差：默认在车辆当前位置取值（lookahead=0，避免与航向偏差重复计入）
        s_eval = projection.s + self.lookahead
        if self.lookahead > 0.0:
            proj = path.project(path.point_at_s(s_eval))
        else:
            proj = projection
        e = proj.signed_distance
        path_yaw = math.atan2(proj.tangent[1], proj.tangent[0])
        psi = float(wrap_to_pi(yaw - path_yaw))
        kappa = path.curvature_at_s(s_eval)
        r_ref = kappa * v_eff                       # 曲率前馈

        # 横向偏差积分（抗饱和：误差过大时不累积，且限幅）
        if abs(e) < 2.0:
            self._integral += e * dt
            self._integral = float(np.clip(self._integral,
                                           -self.integral_limit, self.integral_limit))

        gain = self.gain(speed)
        err = np.array([e, psi, self._r - r_ref])
        r_des = r_ref - float(gain @ err) - self.ki * self._integral
        # 执行器一阶滞后
        self._r += dt / max(t, 1e-3) * (r_des - self._r)

        delta_ref = math.atan(l * self._r / v_eff)
        delta = self._limit(delta_ref, dt)
        return ControllerResult(delta, {"e": e, "psi": psi, "kappa": kappa,
                                        "r_ref": r_ref, "r_des": r_des,
                                        "integral": self._integral})


def build_controller(name: str, params: VehicleParams | None = None,
                     **kwargs) -> BaseController:
    """按名称构造控制器。"""
    table = {"pure_pursuit": PurePursuit, "stanley": Stanley, "lqr": LQR}
    if name not in table:
        raise KeyError(f"未知控制器 '{name}'，可选: {', '.join(table)}")
    return table[name](params=params, **kwargs)
