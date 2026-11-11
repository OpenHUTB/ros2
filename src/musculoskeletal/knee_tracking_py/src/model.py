"""单关节膝关节 + 两条拮抗肌的刚体动力学模型。

坐标约定
--------
* theta = 0 deg：小腿竖直下垂，完全伸展，重力矩为 0。
* theta > 0：屈曲（小腿向后抬起）；theta < 0：过伸。
* 正力矩 = 屈曲方向；负力矩 = 伸展方向。

动力学
------
    I * theta_ddot = tau_muscle - m*g*l*sin(theta) - b*theta_dot
    tau_muscle     = (a_flex - a_ext) * F_max * r * f_l(theta)

重力矩幅值为 m*g*l*sin(theta)。由于屈曲时重力要把小腿拉回竖直位置，
该力矩方向为伸展（负号），所以动力学里写成 -m*g*l*sin(theta)。
这正是“不加重力前馈时会出现稳态下垂”的来源。

肌肉激活
--------
    a_dot = (u - a) / tau_act     (u > a)
    a_dot = (u - a) / tau_deact   (u <= a)

力矩臂 r 恒定，等效于原报告中的 wrap 圆柱；角度很大时也不会出现
力臂趋零的奇异。f_l 默认关闭（=1），此时单肌最大力矩严格等于
F_max * r = 500 * 0.035 = 17.5 N·m。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .config import JointConfig, MuscleConfig


@dataclass(frozen=True)
class KneeState:
    """膝关节状态。角度内部一律用 rad、角速度 rad/s、激活 0..1。"""

    theta: float
    omega: float
    flex: float
    ext: float

    def as_array(self) -> np.ndarray:
        return np.array([self.theta, self.omega, self.flex, self.ext], dtype=float)


class KneeModel:
    """带激活动力学的单关节拮抗肌模型。"""

    def __init__(self, joint: JointConfig, muscle: MuscleConfig) -> None:
        self.joint = joint
        self.muscle = muscle
        self.angle_min = math.radians(joint.angle_min_deg)
        self.angle_max = math.radians(joint.angle_max_deg)
        self.optimal_angle = math.radians(muscle.optimal_angle_deg)

    # -- 基本量 ---------------------------------------------------------
    @property
    def max_muscle_torque(self) -> float:
        """单条肌肉的最大力矩 [N·m]。"""
        return self.muscle.max_force * self.muscle.moment_arm

    def force_length_factor(self, theta: float) -> float:
        """简化力-长度因子；strength=0 时恒为 1。"""
        strength = self.muscle.force_length_strength
        if strength <= 0.0:
            return 1.0
        # 以 +/-90 deg 为归一化尺度，最优角附近为 1，两端略降。
        x = (theta - self.optimal_angle) / (math.pi / 2.0)
        return max(0.2, 1.0 - strength * x * x)

    def gravity_torque(self, theta: float) -> float:
        """重力矩 [N·m]，正角屈曲时指向伸展（负号）。"""
        return -self.joint.gravity_torque * math.sin(theta)

    def muscle_torque(self, state: KneeState) -> float:
        """当前激活下的净肌肉力矩 [N·m]，屈肌为正。"""
        scale = self.max_muscle_torque * self.force_length_factor(state.theta)
        return (state.flex - state.ext) * scale

    def limit_margin_deg(self, theta: float) -> float:
        """到最近限位的距离（度），仅用于展示。"""
        return min(
            math.degrees(theta - self.angle_min),
            math.degrees(self.angle_max - theta),
        )

    # -- 动力学 ---------------------------------------------------------
    def _joint_derivatives(
        self,
        theta: float,
        omega: float,
        a_flex: float,
        a_ext: float,
    ) -> np.ndarray:
        state = KneeState(theta, omega, a_flex, a_ext)
        tau = (
            self.muscle_torque(state)
            + self.gravity_torque(theta)
            - self.joint.damping * omega
        )
        return np.array([omega, tau / self.joint.inertia], dtype=float)

    def derivatives(
        self,
        state: KneeState,
        u_flex: float,
        u_ext: float,
    ) -> np.ndarray:
        """四状态 ODE 的右端项，供分析/测试使用。"""
        theta, omega, a_flex, a_ext = state.as_array()
        d_theta, d_omega = self._joint_derivatives(theta, omega, a_flex, a_ext)
        d_flex = (u_flex - a_flex) / (
            self.muscle.tau_act if u_flex > a_flex else self.muscle.tau_deact
        )
        d_ext = (u_ext - a_ext) / (
            self.muscle.tau_act if u_ext > a_ext else self.muscle.tau_deact
        )
        return np.array([d_theta, d_omega, d_flex, d_ext], dtype=float)

    def _advance_activation(self, a: float, u: float, dt: float) -> float:
        """一阶激活 ODE 的解析解，任意 dt 都不会越界/振荡。"""
        if u > a:
            tau = self.muscle.tau_act
        else:
            tau = self.muscle.tau_deact
        return u + (a - u) * math.exp(-dt / tau)

    def _enforce_limits(self, theta: float, omega: float) -> tuple[float, float]:
        """硬限位：越界时拉回边界，并把继续压向限位的速度清零。"""
        if theta > self.angle_max:
            return self.angle_max, min(omega, 0.0)
        if theta < self.angle_min:
            return self.angle_min, max(omega, 0.0)
        return theta, omega

    def step(
        self,
        state: KneeState,
        u_flex: float,
        u_ext: float,
        dt: float,
    ) -> KneeState:
        """混合积分一步：关节刚体用 RK4，激活用解析指数更新。

        激活时间常数最小只有 0.02 s，若 dt 偏大，把激活也塞进 RK4 会在
        状态越过 0/1 时出现导数符号翻转。这里利用激活方程是线性的，
        直接取精确解，既无条件和稳定又保持物理含义。
        """
        if dt <= 0:
            raise ValueError("dt 必须 > 0")
        u_flex = float(np.clip(u_flex, 0.0, 1.0))
        u_ext = float(np.clip(u_ext, 0.0, 1.0))
        theta, omega, a_flex, a_ext = state.as_array()

        def f(th: float, om: float) -> np.ndarray:
            return self._joint_derivatives(th, om, a_flex, a_ext)

        k1 = f(theta, omega)
        k2 = f(theta + 0.5 * dt * k1[0], omega + 0.5 * dt * k1[1])
        k3 = f(theta + 0.5 * dt * k2[0], omega + 0.5 * dt * k2[1])
        k4 = f(theta + dt * k3[0], omega + dt * k3[1])

        new_theta = theta + dt / 6.0 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0])
        new_omega = omega + dt / 6.0 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])
        if not math.isfinite(new_theta) or not math.isfinite(new_omega):
            raise FloatingPointError("积分出现 NaN/Inf，请检查输入或减小 dt")

        new_theta, new_omega = self._enforce_limits(new_theta, new_omega)
        new_flex = float(np.clip(self._advance_activation(a_flex, u_flex, dt), 0.0, 1.0))
        new_ext = float(np.clip(self._advance_activation(a_ext, u_ext, dt), 0.0, 1.0))
        return KneeState(new_theta, new_omega, new_flex, new_ext)
