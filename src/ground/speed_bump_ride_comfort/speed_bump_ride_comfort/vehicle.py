"""两自由度四分之一车模型与定步长 RK4 积分。

状态取相对静态平衡位置的位移：``[z_s, z_s_dot, z_u, z_u_dot]``，其中 ``z_s`` 为
车身（悬挂质量）垂向位移，``z_u`` 为非悬挂质量位移，路面激励为 ``z_r``。

运动方程（线性悬架 + 线性轮胎）::

    m_s * z_s_ddot = -k_s * (z_s - z_u) - c_s * (z_s_dot - z_u_dot)
    m_u * z_u_ddot =  k_s * (z_s - z_u) + c_s * (z_s_dot - z_u_dot) - k_t * (z_u - z_r)

模型只关心垂向平顺性问题，因此忽略了俯仰、侧倾、悬架限位非线性与减振器速度特性，
这些简化在文档的"已知限制"中写明。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .bump import SpeedBump

#: 标准重力加速度（m/s²）。
GRAVITY = 9.80665


@dataclass(frozen=True)
class Passage:
    """一次减速带通过的完整时间历程，数组长度一致。"""

    time_s: np.ndarray
    road_height_m: np.ndarray
    sprung_position_m: np.ndarray
    sprung_accel_mps2: np.ndarray
    travel_m: np.ndarray
    tire_load_ratio: np.ndarray
    speed_mps: float


class QuarterCar:
    """四分之一车模型参数与积分。

    默认参数对应一辆总质量约 1.5 t 的乘用车单角（车身 350 kg + 非悬挂 45 kg），
    车身偏频约 1.47 Hz、车轮偏频约 12.6 Hz，属于常见乘用车的量级。
    """

    def __init__(
        self,
        sprung_mass_kg: float = 350.0,
        unsprung_mass_kg: float = 45.0,
        suspension_stiffness_n_per_m: float = 30000.0,
        suspension_damping_n_s_per_m: float = 1800.0,
        tire_stiffness_n_per_m: float = 250000.0,
        travel_limit_m: float = 0.090,
    ) -> None:
        for name, value in (
            ("sprung_mass_kg", sprung_mass_kg),
            ("unsprung_mass_kg", unsprung_mass_kg),
            ("suspension_stiffness_n_per_m", suspension_stiffness_n_per_m),
            ("tire_stiffness_n_per_m", tire_stiffness_n_per_m),
        ):
            if not value > 0.0:
                raise ValueError(f"{name} must be positive")
        if suspension_damping_n_s_per_m < 0.0:
            raise ValueError("suspension_damping_n_s_per_m must not be negative")
        self.sprung_mass_kg = float(sprung_mass_kg)
        self.unsprung_mass_kg = float(unsprung_mass_kg)
        self.suspension_stiffness_n_per_m = float(suspension_stiffness_n_per_m)
        self.suspension_damping_n_s_per_m = float(suspension_damping_n_s_per_m)
        self.tire_stiffness_n_per_m = float(tire_stiffness_n_per_m)
        self.travel_limit_m = float(travel_limit_m)

    # ------------------------------------------------------------------ 参数推导
    @property
    def body_natural_frequency_hz(self) -> float:
        """车身跳动偏频 ``sqrt(k_s / m_s) / 2π``。"""
        return float(
            np.sqrt(self.suspension_stiffness_n_per_m / self.sprung_mass_kg)
            / (2.0 * np.pi)
        )

    @property
    def wheel_natural_frequency_hz(self) -> float:
        """车轮跳动偏频 ``sqrt((k_s + k_t) / m_u) / 2π``。"""
        total = self.suspension_stiffness_n_per_m + self.tire_stiffness_n_per_m
        return float(np.sqrt(total / self.unsprung_mass_kg) / (2.0 * np.pi))

    @property
    def damping_ratio(self) -> float:
        """车身模态阻尼比 ``c_s / (2 sqrt(k_s m_s))``。"""
        critical = 2.0 * np.sqrt(
            self.suspension_stiffness_n_per_m * self.sprung_mass_kg
        )
        return float(self.suspension_damping_n_s_per_m / critical)

    @property
    def static_wheel_load_n(self) -> float:
        """单角静态轮荷 ``(m_s + m_u) g``。"""
        return float((self.sprung_mass_kg + self.unsprung_mass_kg) * GRAVITY)

    # ------------------------------------------------------------------ 动力学
    def derivatives(self, state: np.ndarray, road_height_m: float) -> np.ndarray:
        """状态导数，输入为状态向量与当前路面高度。"""
        z_s, v_s, z_u, v_u = (float(value) for value in state)
        ks = self.suspension_stiffness_n_per_m
        cs = self.suspension_damping_n_s_per_m
        kt = self.tire_stiffness_n_per_m
        spring_force = ks * (z_s - z_u) + cs * (v_s - v_u)
        accel_sprung = -spring_force / self.sprung_mass_kg
        accel_unsprung = (
            spring_force - kt * (z_u - road_height_m)
        ) / self.unsprung_mass_kg
        return np.array([v_s, accel_sprung, v_u, accel_unsprung])

    def sprung_acceleration(self, state: np.ndarray, road_height_m: float) -> float:
        """车身垂向加速度（m/s²），供指标与绘图使用。"""
        return float(self.derivatives(state, road_height_m)[1])

    def simulate(
        self,
        bump: SpeedBump,
        speed_mps: float,
        dt: float = 1.0e-3,
        approach_s: float = 0.2,
        settle_s: float = 2.0,
    ) -> Passage:
        """从减速带前沿前 ``approach_s`` 开始积分到通过后再 ``settle_s``。

        ``t = 0`` 对应车轮到达减速带前沿的时刻。``approach_s`` 段用于给出
        零基线，``settle_s`` 段用于观察车身自由度回落，两者都计入分析窗口。
        """
        if not speed_mps > 0.0:
            raise ValueError("speed_mps must be positive")
        if not dt > 0.0:
            raise ValueError("dt must be positive")
        if approach_s < 0.0 or settle_s < 0.0:
            raise ValueError("approach_s and settle_s must not be negative")

        contact_s = bump.duration_at(speed_mps)
        start_s = -float(approach_s)
        end_s = contact_s + float(settle_s)
        steps = int(np.floor((end_s - start_s) / dt)) + 1
        time = start_s + dt * np.arange(steps)

        def road_at(t: float) -> float:
            return float(bump.profile_time(t, speed_mps))

        state = np.zeros(4)
        position = np.zeros((steps, 4))
        road = np.zeros(steps)
        for index, t in enumerate(time):
            road[index] = road_at(t)
            position[index] = state
            if index == steps - 1:
                break
            z_r = road[index]
            z_r_mid = road_at(t + 0.5 * dt)
            z_r_next = road_at(t + dt)
            k1 = self.derivatives(state, z_r)
            k2 = self.derivatives(state + 0.5 * dt * k1, z_r_mid)
            k3 = self.derivatives(state + 0.5 * dt * k2, z_r_mid)
            k4 = self.derivatives(state + dt * k3, z_r_next)
            state = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        z_s = position[:, 0]
        z_u = position[:, 2]
        accel = np.array(
            [
                -(
                    self.suspension_stiffness_n_per_m * (zs - zu)
                    + self.suspension_damping_n_s_per_m * (vs - vu)
                )
                / self.sprung_mass_kg
                for zs, zu, vs, vu in zip(z_s, z_u, position[:, 1], position[:, 3])
            ]
        )
        contact_force = self.tire_stiffness_n_per_m * (z_u - road)
        tire_load_ratio = 1.0 + contact_force / self.static_wheel_load_n
        return Passage(
            time_s=time,
            road_height_m=road,
            sprung_position_m=z_s,
            sprung_accel_mps2=accel,
            travel_m=z_s - z_u,
            tire_load_ratio=tire_load_ratio,
            speed_mps=float(speed_mps),
        )
