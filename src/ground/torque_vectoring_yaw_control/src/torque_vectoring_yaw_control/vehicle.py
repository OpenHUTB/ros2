"""车辆参数与轮胎模型（含附着椭圆）。

坐标约定：``x`` 车头方向、``y`` 车身左侧、``psi`` 航向（逆时针为正）。
轮胎侧偏力用线性段，超过附着极限按 ``mu*Fz`` 截断；纵向力则受附着椭圆约束：

.. math::

    F_{x,max} = \\sqrt{(\\mu F_z)^2 - F_y^2}

这条式子就是扭矩矢量分配的硬边界——**先有侧向力，纵向力就没那么多额度了**。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

G = 9.80665

__all__ = ["G", "VehicleSpec", "WheelState"]


@dataclass(frozen=True)
class VehicleSpec:
    """1.6 t 级双电机四驱试验车。"""

    mass: float = 1650.0
    iz: float = 2600.0            # 横摆转动惯量，kg·m²
    wheelbase: float = 2.72
    a: float = 1.28               # 质心到前轴，m
    track: float = 1.56
    cg_height: float = 0.55
    wheel_radius: float = 0.33
    mu: float = 0.95
    cf: float = 90000.0           # 前轴侧偏刚度，N/rad（整轴）
    cr: float = 110000.0          # 后轴侧偏刚度，N/rad（整轴）
    cd_a: float = 0.42            # 0.5*rho*Cd*A，N/(m/s)²
    # 轮端等效扭矩（含减速比），不是电机轴扭矩。四轮合计约 0.67 g，
    # 这样附着才是多数工况下的紧约束——否则限扭先饱和，路面差异根本体现不出来。
    motor_torque_max: float = 900.0
    steer_max_deg: float = 32.0
    steer_rate_deg: float = 380.0

    @property
    def b(self) -> float:
        return self.wheelbase - self.a

    @property
    def weight(self) -> float:
        return self.mass * G

    @property
    def front_share(self) -> float:
        return self.b / self.wheelbase

    def static_loads(self) -> tuple[float, float]:
        """前后轴静态垂向载荷，N。"""
        return self.weight * self.front_share, self.weight * (1.0 - self.front_share)

    def axle_loads(self, ax: float = 0.0) -> tuple[float, float]:
        """含纵向载荷转移的前后轴载荷，N。``ax > 0`` 为加速（前轴减载）。"""
        f_static, r_static = self.static_loads()
        transfer = self.mass * ax * self.cg_height / self.wheelbase
        f = max(1.0, f_static - transfer)
        r = max(1.0, r_static + transfer)
        return f, r

    def understeer_gradient(self) -> float:
        """不足转向梯度 K，s²/m。正值不足转向，负值过多转向。

        ``K = (m/L)·(b/Cf − a/Cr)``：前后轴侧偏刚度的位置不能对调，
        否则"前轴更软"会被判成过多转向，参考横摆率整个反过来。
        """
        return self.mass / self.wheelbase * (self.b / self.cf - self.a / self.cr)

    def yaw_rate_reference(self, u: float, steer: float) -> float:
        """线性二自由度模型的稳态横摆角速度，rad/s。"""
        denom = self.wheelbase + self.understeer_gradient() * u * u
        if denom <= 1e-6:
            return math.copysign(math.pi, steer)
        return u * steer / denom

    def yaw_rate_limit(self, u: float, mu: float | None = None) -> float:
        """附着极限给出的横摆角速度上限 |r| ≤ μg/u。"""
        mu = self.mu if mu is None else mu
        return mu * G / max(u, 0.5)

    def force_cap(self, load: float, fy: float, mu: float | None = None) -> float:
        """附着椭圆给出的纵向力上限（非负）。"""
        mu = self.mu if mu is None else mu
        grip = mu * load
        return math.sqrt(max(grip * grip - fy * fy, 0.0))

    def motor_force_max(self) -> float:
        """单轮电机对应的最大纵向力，N。"""
        return self.motor_torque_max / self.wheel_radius

    def axle_forces(self, u: float, v: float, r: float, steer: float,
                    ax: float = 0.0) -> tuple[float, float, float, float]:
        """当前状态下的前后轴侧偏力与垂向载荷 ``(Fyf, Fyr, Fzf, Fzr)``。

        轮胎侧偏力取线性段并在 ``μFz`` 处截断——推头与甩尾都是这个截断的
        自然结果，不需要额外的"是否失稳"判据。节点和仿真共用这一处，
        避免在线估计与离线验证用两套公式而对不上。
        """
        fz_f, fz_r = self.axle_loads(ax)
        u_safe = max(u, 1.0)
        alpha_f = steer - math.atan2(v + self.a * r, u_safe)
        alpha_r = -math.atan2(v - self.b * r, u_safe)
        fy_f = max(-self.mu * fz_f, min(self.mu * fz_f, self.cf * alpha_f))
        fy_r = max(-self.mu * fz_r, min(self.mu * fz_r, self.cr * alpha_r))
        return fy_f, fy_r, fz_f, fz_r


@dataclass
class WheelState:
    """四个车轮的纵向力与可用额度，顺序固定为 左前、右前、左后、右后。"""

    fx: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    cap: list[float] = field(default_factory=lambda: [1.0, 1.0, 1.0, 1.0])
    load: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])

    @staticmethod
    def moment_arms(track: float) -> list[float]:
        """纵向力对横摆的力臂：左侧推 +y 产生负横摆，右侧相反。"""
        half = track / 2.0
        return [-half, half, -half, half]

    @property
    def total(self) -> float:
        return sum(self.fx)

    @property
    def utilization(self) -> float:
        """最大附着利用率 |Fx|/cap，1.0 表示打滑边缘。"""
        used = [abs(f) / c for f, c in zip(self.fx, self.cap) if c > 1e-6]
        return max(used) if used else 0.0
