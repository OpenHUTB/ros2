"""车辆模型、转向执行器和纵向速度策略。

默认是后轴参考点的运动学自行车模型（RK4，dt=0.02 s）：
    x_dot = v cos(psi)
    y_dot = v sin(psi)
    psi_dot = v tan(delta) / L
另外提供可选的线性轮胎动态自行车模型，用于 `--model dynamic` 对照。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VehicleParams:
    """车辆参数。长度单位 m，角度单位 rad。"""

    wheelbase: float = 2.7
    max_steer: float = np.deg2rad(35.0)
    max_steer_rate: float = 1.20  # rad/s
    tau_speed: float = 0.70       # 纵向一阶响应时间常数
    max_accel: float = 3.5        # m/s^2
    max_decel: float = 4.5        # m/s^2
    a_lat_max: float = 4.0        # 曲率限速对应的最大横向加速度

    # 动态模型参数（仅 --model dynamic 使用）
    mass: float = 1500.0
    inertia: float = 2500.0
    lf: float = 1.20
    lr: float = 1.50
    cf: float = 80000.0
    cr: float = 85000.0
    tire_alpha_limit: float = np.deg2rad(8.0)
    mu: float = 0.9
    gravity: float = 9.81


class SteeringActuator:
    """同时执行转角限幅和转角速率限幅。"""

    def __init__(self, params: VehicleParams, initial: float = 0.0):
        self.params = params
        self.delta = float(np.clip(initial, -params.max_steer, params.max_steer))

    def reset(self, initial: float = 0.0):
        self.delta = float(np.clip(initial, -self.params.max_steer, self.params.max_steer))

    def command(self, desired: float, dt: float) -> float:
        if not np.isfinite(desired):
            desired = 0.0
        desired = float(np.clip(desired, -self.params.max_steer, self.params.max_steer))
        max_step = self.params.max_steer_rate * dt
        delta = float(np.clip(desired, self.delta - max_step, self.delta + max_step))
        self.delta = float(np.clip(delta, -self.params.max_steer, self.params.max_steer))
        return self.delta


def curvature_speed_limit(kappa: float, cruise_speed: float, a_lat_max: float) -> float:
    """由曲率给出的速度上限；直道返回巡航速度。"""
    kappa = abs(float(kappa))
    if kappa < 1e-9:
        return float(cruise_speed)
    return float(min(cruise_speed, np.sqrt(a_lat_max / kappa)))


def longitudinal_target(s, path, cruise_speed: float, params: VehicleParams) -> float:
    pose = path.pose_at(s)
    return curvature_speed_limit(pose.kappa, cruise_speed, params.a_lat_max)


def _speed_derivative(v: float, v_target: float, params: VehicleParams) -> float:
    raw = (v_target - v) / params.tau_speed
    return float(np.clip(raw, -params.max_decel, params.max_accel))


def kinematic_derivative(state, delta, v_target, params: VehicleParams):
    x, y, psi, v = state
    v = max(float(v), 0.0)
    return np.array(
        [
            v * np.cos(psi),
            v * np.sin(psi),
            v * np.tan(delta) / params.wheelbase,
            _speed_derivative(v, v_target, params),
        ],
        dtype=float,
    )


def rk4_step(state, delta: float, v_target: float, dt: float, params: VehicleParams):
    """固定转角的四阶 Runge-Kutta 积分。"""
    state = np.asarray(state, dtype=float)
    k1 = kinematic_derivative(state, delta, v_target, params)
    k2 = kinematic_derivative(state + 0.5 * dt * k1, delta, v_target, params)
    k3 = kinematic_derivative(state + 0.5 * dt * k2, delta, v_target, params)
    k4 = kinematic_derivative(state + dt * k3, delta, v_target, params)
    new_state = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    new_state[3] = max(0.0, new_state[3])
    return new_state


def dynamic_derivative(state, delta, v_target, params: VehicleParams):
    """线性轮胎动态自行车模型（车身坐标 x 向前、y 向左）。"""
    x, y, psi, vx, vy, r = state
    vx_safe = max(float(vx), 0.8)
    cos_d = np.cos(delta)
    sin_d = np.sin(delta)

    alpha_f = delta - np.arctan2(vy + params.lf * r, vx_safe)
    alpha_r = -np.arctan2(vy - params.lr * r, vx_safe)
    alpha_f = float(np.clip(alpha_f, -params.tire_alpha_limit, params.tire_alpha_limit))
    alpha_r = float(np.clip(alpha_r, -params.tire_alpha_limit, params.tire_alpha_limit))
    f_max = params.mu * params.mass * params.gravity * 0.5
    fyf = float(np.clip(params.cf * alpha_f, -f_max, f_max))
    fyr = float(np.clip(params.cr * alpha_r, -f_max, f_max))

    vx_dot = _speed_derivative(vx, v_target, params)
    vy_dot = (fyf * cos_d + fyr) / params.mass - vx * r
    r_dot = (params.lf * fyf * cos_d - params.lr * fyr) / params.inertia
    return np.array(
        [
            vx * np.cos(psi) - vy * np.sin(psi),
            vx * np.sin(psi) + vy * np.cos(psi),
            r,
            vx_dot,
            vy_dot,
            r_dot,
        ],
        dtype=float,
    )


def rk4_step_dynamic(state, delta: float, v_target: float, dt: float, params: VehicleParams):
    state = np.asarray(state, dtype=float)
    k1 = dynamic_derivative(state, delta, v_target, params)
    k2 = dynamic_derivative(state + 0.5 * dt * k1, delta, v_target, params)
    k3 = dynamic_derivative(state + 0.5 * dt * k2, delta, v_target, params)
    k4 = dynamic_derivative(state + dt * k3, delta, v_target, params)
    new_state = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    new_state[3] = max(0.0, new_state[3])
    return new_state


def initial_dynamic_state(kinematic_state):
    x, y, psi, v = np.asarray(kinematic_state, dtype=float)
    return np.array([x, y, psi, v, 0.0, 0.0], dtype=float)
