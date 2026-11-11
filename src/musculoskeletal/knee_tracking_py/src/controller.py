"""PD 控制器 + 重力矩前馈，以及“力矩 -> 肌肉激活”的归一化映射。

    e   = theta_des - theta                    （内部用 rad）
    tau = kp * e - kd * theta_dot - g_ff * tau_g(theta)   （N·m）

其中 tau_g 是重力矩（本模型为 `-m*g*l*sin(theta)`，屈曲为正）。不加前馈
（g_ff = 0）时，PD 必须靠一个持续存在的误差 e 来平衡重力，表现为稳态下垂：

    e_ss ≈ tau_g / kp

theta = 50° 时 tau_g ≈ 3.45 N·m、kp = 30 N·m/rad，理论下垂约 6.6°；实测
−5.97°。打开前馈后同一增益的稳态误差降到 1e−4 度量级。

映射规则：
    tau >= 0：flexor = clamp(tau / tau_max, 0, 1)，extensor = 0
    tau <  0：extensor = clamp(-tau / tau_max, 0, 1)，flexor = 0

其中 tau_max = F_max * r = 17.5 N·m。两条肌肉不会同时激活。

接口约定：theta/theta_des 用 rad，omega 用 rad/s；返回值中的 error_deg 用度。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

from .config import ControllerConfig


@dataclass(frozen=True)
class ControlCommand:
    """一次控制计算的输出。"""

    flex: float
    ext: float
    tau_pd: float
    error_deg: float
    valid: bool
    reason: str
    tau_gravity: float = 0.0


class PDController:
    """带限位保护、非法输入保护与重力前馈的 PD 控制器。"""

    def __init__(
        self,
        config: ControllerConfig,
        max_torque: float,
        angle_min_deg: float,
        angle_max_deg: float,
        gravity_torque_fn: Callable[[float], float] | None = None,
    ) -> None:
        if max_torque <= 0:
            raise ValueError("max_torque 必须 > 0")
        if angle_min_deg >= angle_max_deg:
            raise ValueError("关节下限必须小于上限")
        self.config = config
        self.max_torque = float(max_torque)
        self.angle_min = math.radians(angle_min_deg)
        self.angle_max = math.radians(angle_max_deg)
        self.epsilon = math.radians(config.limit_epsilon_deg)
        self.gravity_torque_fn = gravity_torque_fn
        self.gravity_gain = float(config.gravity_feedforward)
        if self.gravity_gain < 0.0:
            raise ValueError("gravity_feedforward 必须 >= 0")

    def gravity_compensation(self, theta: float) -> float:
        """需要额外施加的补偿力矩（与重力矩反向）。"""
        if self.gravity_torque_fn is None or self.gravity_gain <= 0.0:
            return 0.0
        return -self.gravity_gain * float(self.gravity_torque_fn(theta))

    def compute(
        self,
        theta: float,
        omega: float,
        theta_des: float,
    ) -> ControlCommand:
        """根据当前状态与目标角（rad）计算肌肉激活指令。"""
        values = (theta, omega, theta_des)
        numeric = all(
            isinstance(v, (int, float)) and not isinstance(v, bool)
            for v in values
        )
        if not numeric or not all(math.isfinite(float(v)) for v in values):
            return ControlCommand(0.0, 0.0, 0.0, 0.0, False, "invalid_input")

        if not (self.angle_min <= theta_des <= self.angle_max):
            # 目标超出机械限位视为非法输入，直接放松肌肉。
            return ControlCommand(0.0, 0.0, 0.0, 0.0, False, "invalid_target")

        error = theta_des - theta
        tau_fb = self.config.kp * error - self.config.kd * omega
        tau_ff = self.gravity_compensation(theta)
        tau = tau_fb + tau_ff
        if tau >= 0.0:
            u_flex = min(max(tau / self.max_torque, 0.0), 1.0)
            u_ext = 0.0
        else:
            u_flex = 0.0
            u_ext = min(max(-tau / self.max_torque, 0.0), 1.0)

        reason = "ok"
        if tau >= 0.0 and tau / self.max_torque > 1.0:
            reason = "flexor_saturated"
        elif tau < 0.0 and -tau / self.max_torque > 1.0:
            reason = "extensor_saturated"

        # 安全保护 1：已到 90 deg 上限却仍要求屈曲 -> 停屈肌。
        if theta >= self.angle_max - self.epsilon and tau > 0.0 and u_flex > 0.0:
            u_flex = 0.0
            reason = "upper_limit_flexor_off"

        # 安全保护 2：已到 -15 deg 下限却仍要求伸展 -> 停伸肌。
        if theta <= self.angle_min + self.epsilon and tau < 0.0 and u_ext > 0.0:
            u_ext = 0.0
            reason = "lower_limit_extensor_off"

        return ControlCommand(
            flex=u_flex,
            ext=u_ext,
            tau_pd=tau,
            error_deg=math.degrees(error),
            valid=True,
            reason=reason,
            tau_gravity=tau_ff,
        )
