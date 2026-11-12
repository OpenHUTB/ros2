"""速度/航向双环控制，输出（纵向力, 艏摇力矩）指令。

内环是艏摇角速度，外环是航向；纵向用"比例 + 阻尼前馈"，前馈项直接抵消船体阻力，
否则积分项会在低速时累积出明显的稳态误差。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .geometry import clamp, wrap_angle


@dataclass(frozen=True)
class ControlConfig:
    """控制参数。"""

    surge_gain: float = 4.5            # N per (m/s)
    surge_feedforward_drag: float = 6.0  # 与船体纵向阻尼一致，用于阻力前馈
    surge_deadband: float = 0.04       # 期望速度低于此值且实测也低时不再给推力
    heading_gain: float = 1.60         # (rad/s) per rad
    heading_rate_limit: float = 0.55   # 期望艏摇角速度上限 rad/s
    heading_deadband_deg: float = 2.0  # 航向误差死区，抑制低速抖动
    yaw_gain: float = 2.2              # N·m per (rad/s)
    yaw_deadband: float = 0.02


@dataclass(frozen=True)
class WrenchCommand:
    """控制器的输出：纵向力与艏摇力矩。"""

    surge: float
    yaw: float
    heading_error: float = 0.0
    active: bool = True


class StationKeepingController:
    """把制导指令转成力/力矩，含死区与抗抖处理。"""

    def __init__(self, config: ControlConfig | None = None) -> None:
        self.config = config or ControlConfig()

    def update(
        self,
        u_cmd: float,
        psi_cmd: float,
        psi: float,
        u: float,
        r: float,
        allow_thrust: bool = True,
    ) -> WrenchCommand:
        cfg = self.config
        heading_error = wrap_angle(psi_cmd - psi)

        # 纵向：比例 + 阻力前馈；期望与实测都接近 0 时彻底关断，省电且减少抖动
        if not allow_thrust or (abs(u_cmd) < cfg.surge_deadband and abs(u) < cfg.surge_deadband):
            surge = 0.0
        else:
            surge = cfg.surge_gain * (u_cmd - u) + cfg.surge_feedforward_drag * u_cmd

        # 航向：外环限速，内环阻尼
        if abs(math.degrees(heading_error)) < cfg.heading_deadband_deg and abs(u) < 0.08:
            yaw = 0.0
        else:
            rate_cmd = clamp(cfg.heading_gain * heading_error, -cfg.heading_rate_limit, cfg.heading_rate_limit)
            yaw_error = rate_cmd - r
            yaw = 0.0 if abs(yaw_error) < cfg.yaw_deadband else cfg.yaw_gain * yaw_error

        active = abs(surge) > 1e-9 or abs(yaw) > 1e-9
        return WrenchCommand(surge=surge, yaw=yaw, heading_error=heading_error, active=active)
