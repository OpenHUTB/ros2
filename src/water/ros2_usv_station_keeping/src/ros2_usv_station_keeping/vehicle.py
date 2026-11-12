"""欠驱动 USV 的三自由度平面运动模型（纯 NumPy，可在无 ROS 环境下运行）。

坐标系约定（与 ROS REP-103 一致，z 轴向上）：

- 世界系 ``x`` 向东、``y`` 向北，航向角 ``psi`` 为船头相对世界 x 轴的逆时针夹角；
- 船体系 ``u`` 沿船头向前、``v`` 指向左舷（不是右舷），``r`` 为艏摇角速度。

关键约束：船体**没有侧向推进器**，横向运动只能来自转弯时的离心侧滑和流场拖曳。
这正是定点保持的难点所在——想往右平移 1 米，只能先转向再前进，或者靠"斜着顶流"。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

from .geometry import body_to_world, clamp, world_to_body


@dataclass(frozen=True)
class VehicleParams:
    """船体参数（默认值对应一艘 1.2 m 长的双推进器小型 USV）。"""

    mass_surge: float = 15.0       # 含附加质量的纵向质量 kg
    mass_sway: float = 25.0        # 横向质量（附加质量更大）
    inertia_yaw: float = 1.2       # 艏摇转动惯量 kg·m^2
    damp_surge: float = 6.0        # 纵向线性阻尼 N·s/m
    damp_sway: float = 25.0        # 横向线性阻尼 N·s/m
    damp_yaw: float = 1.5          # 艏摇阻尼 N·m·s

    thrust_max: float = 4.0        # 单侧最大正推力 N
    thrust_min: float = -2.4       # 单侧最大反推力 N（倒车推力通常小于正推）
    thruster_spacing: float = 0.5  # 两推进器横向间距 m
    thrust_tau: float = 0.12       # 推进器一阶响应时间常数 s

    @property
    def max_surge_speed(self) -> float:
        """两推进器满推时的稳态直航速度。"""
        return 2.0 * self.thrust_max / self.damp_surge


@dataclass
class VehicleState:
    """船体状态。``x``/``y`` 为世界坐标，``u``/``v`` 为船体速度。"""

    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0
    u: float = 0.0
    v: float = 0.0
    r: float = 0.0

    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.psi

    def copy(self) -> "VehicleState":
        return replace(self)

    def world_velocity(self) -> tuple[float, float]:
        """船体速度在世界系下的分量（不含流）。"""
        return body_to_world(self.psi, self.u, self.v)


@dataclass
class ActuatorState:
    """推进器的实际输出（含一阶滞后），用于模拟真实响应。"""

    left: float = 0.0
    right: float = 0.0

    def step(self, cmd_left: float, cmd_right: float, tau: float, dt: float) -> None:
        if tau <= 1e-6:
            self.left, self.right = cmd_left, cmd_right
            return
        alpha = min(1.0, dt / tau)
        self.left += alpha * (cmd_left - self.left)
        self.right += alpha * (cmd_right - self.right)

    def thrusters(self) -> tuple[float, float]:
        return self.left, self.right


def relative_velocity(state: VehicleState, current_world: tuple[float, float]) -> tuple[float, float]:
    """水相对速度（船体系）。流场 ``current_world`` 是世界系下的水流速度。"""
    cx, cy = world_to_body(state.psi, current_world[0], current_world[1])
    return state.u - cx, state.v - cy


def thrust_to_wrench(
    thrust_left: float,
    thrust_right: float,
    params: VehicleParams,
) -> tuple[float, float]:
    """把左右推力合成为（纵向力 X，艏摇力矩 N）。"""
    surge = thrust_left + thrust_right
    yaw = (thrust_right - thrust_left) * params.thruster_spacing * 0.5
    return surge, yaw


def integrate(
    state: VehicleState,
    params: VehicleParams,
    thrust_left: float,
    thrust_right: float,
    current_world: tuple[float, float],
    dt: float,
) -> VehicleState:
    """按半隐式欧拉推进一步，返回新状态（不修改入参）。"""
    if dt <= 0.0:
        raise ValueError("dt 必须为正")

    u_rel, v_rel = relative_velocity(state, current_world)
    force_x, torque_n = thrust_to_wrench(thrust_left, thrust_right, params)

    acc_surge = (force_x - params.damp_surge * abs(u_rel) * (1.0 if u_rel >= 0 else -1.0)) / params.mass_surge
    # 侧向：阻尼 + 转弯离心项（m_u * u * r 把船甩向外侧）
    acc_sway = (-params.damp_sway * v_rel - params.mass_surge * state.u * state.r) / params.mass_sway
    acc_yaw = (torque_n - params.damp_yaw * state.r) / params.inertia_yaw

    new_u = state.u + acc_surge * dt
    new_v = state.v + acc_sway * dt
    new_r = state.r + acc_yaw * dt

    vx, vy = body_to_world(state.psi, new_u, new_v)
    new_x = state.x + vx * dt
    new_y = state.y + vy * dt
    new_psi = state.psi + new_r * dt

    return VehicleState(x=new_x, y=new_y, psi=new_psi, u=new_u, v=new_v, r=new_r)


def actuator_limits(params: VehicleParams) -> tuple[float, float]:
    return params.thrust_min, params.thrust_max


def clamp_thrust(value: float, params: VehicleParams) -> float:
    return clamp(value, params.thrust_min, params.thrust_max)


def surge_speed_for_thrust(thrust_total: float, params: VehicleParams) -> float:
    """给定总推力估算稳态纵向速度（用于制导前馈）。"""
    return thrust_total / params.damp_surge


def turning_radius(speed: float, yaw_rate: float) -> float:
    """瞬时转弯半径，``yaw_rate`` 接近 0 时返回 ``inf``。"""
    if abs(yaw_rate) < 1e-6:
        return math.inf
    return abs(speed / yaw_rate)
