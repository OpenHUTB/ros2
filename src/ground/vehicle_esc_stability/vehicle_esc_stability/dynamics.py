"""双轨车辆平面动力学：轮胎力、载荷转移与 RK4 积分。

模型自由度：x、y、横摆角 ψ、纵向车速 u、侧向车速 v、横摆角速度 r。
四个车轮各自算侧偏角、侧向力与制动力，再合成到车体上，
所以左右附着不同（对开路面）时能自然产生横摆力矩。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .params import (
    ControlInput,
    TireSpec,
    VehicleSpec,
    VehicleState,
    WheelState,
)


@dataclass(frozen=True)
class RoadSpec:
    """路面附着。左右分别给，用来构造对开路面（μ-split）。"""

    mu_left: float = 0.90
    mu_right: float = 0.90

    def mu_at(self, index: int) -> float:
        # 0 左前、1 右前、2 左后、3 右后
        return self.mu_left if index in (0, 2) else self.mu_right

    @property
    def mean(self) -> float:
        return 0.5 * (self.mu_left + self.mu_right)


@dataclass
class ForceBundle:
    """一次受力计算的结果。"""

    fx_body: float = 0.0
    fy_body: float = 0.0
    mz_body: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    wheels: tuple[WheelState, ...] = ()
    drag: float = 0.0


def _load_transfer(spec: VehicleSpec, ax: float, ay: float) -> tuple[float, float, float, float]:
    """准静态载荷转移，返回四条轮胎的法向力 (FL, FR, RL, RR)。"""
    total = spec.weight
    base_front = total * spec.cg_to_rear / spec.wheelbase
    base_rear = total * spec.cg_to_front / spec.wheelbase
    # 加速时载荷前移量 m·ax·h/L：加速为正，压向后轴
    shift = spec.mass * ax * spec.cg_height / spec.wheelbase
    front_axle = base_front - shift
    rear_axle = base_rear + shift

    front_axle = max(front_axle, 0.0)
    rear_axle = max(rear_axle, 0.0)

    # 侧向加速度为正（向左）时，载荷压向右侧车轮
    lateral_total = spec.mass * ay * spec.cg_height
    front_shift = lateral_total * spec.roll_split_front / spec.track_front
    rear_shift = lateral_total * (1.0 - spec.roll_split_front) / spec.track_rear

    fl = max(front_axle / 2.0 - front_shift, 0.0)
    fr = max(front_axle / 2.0 + front_shift, 0.0)
    rl = max(rear_axle / 2.0 - rear_shift, 0.0)
    rr = max(rear_axle / 2.0 + rear_shift, 0.0)
    return fl, fr, rl, rr


def wheel_forces(
    state: VehicleState,
    control: ControlInput,
    spec: VehicleSpec,
    tire: TireSpec,
    road: RoadSpec,
    ax_prev: float = 0.0,
    ay_prev: float = 0.0,
) -> ForceBundle:
    """按当前状态算四条轮胎的力，并合成车体合力与横摆力矩。"""
    normal = _load_transfer(spec, ax_prev, ay_prev)
    positions = spec.wheel_positions()

    fx_total = 0.0
    fy_total = 0.0
    mz_total = 0.0
    wheels: list[WheelState] = []

    for index, ((wx, wy, is_front), fz) in enumerate(zip(positions, normal)):
        steer = control.steer if is_front else 0.0
        cos_d, sin_d = math.cos(steer), math.sin(steer)

        # 车轮处的速度（车体坐标）
        vx_wheel = state.u - state.r * wy
        vy_wheel = state.v + state.r * wx

        # 转到轮胎坐标
        vx_tire = vx_wheel * cos_d + vy_wheel * sin_d
        vy_tire = -vx_wheel * sin_d + vy_wheel * cos_d

        if abs(vx_tire) < 0.25:
            vx_tire = 0.25 if vx_tire >= 0.0 else -0.25
        slip = math.atan2(vy_tire, abs(vx_tire))

        mu = road.mu_at(index)
        fy = tire.lateral(slip, mu, fz)

        brake = control.wheel_brake[index] if control.wheel_brake else 0.0
        brake = min(max(brake, 0.0), 1.0)
        # 理想 ABS：纵向力只用到附着的 90%，给侧向留出余量
        # 制动力方向与车轮滚动方向相反，倒车时不能继续把车往后推
        rolling = 1.0 if vx_tire >= 0.0 else -1.0
        brake_force = rolling * brake * min(spec.brake_force_max, tire.abs_ratio * mu * fz)

        drive = 0.0
        if is_front and control.throttle > 0.0:
            drive = control.throttle * spec.drive_force_max * max(0.0, 1.0 - state.u / 60.0)
        fx = drive - brake_force

        # 附着椭圆：纵向力优先，剩余抓地力给侧向
        limit = mu * fz
        saturated = False
        if fx * fx + fy * fy > limit * limit:
            saturated = True
            remaining = limit * limit - fx * fx
            fy_allowed = math.sqrt(remaining) if remaining > 0.0 else 0.0
            fy = math.copysign(min(abs(fy), fy_allowed), fy)

        # 转回车体坐标
        fx_body = fx * cos_d - fy * sin_d
        fy_body = fx * sin_d + fy * cos_d

        fx_total += fx_body
        fy_total += fy_body
        mz_total += wx * fy_body - wy * fx_body

        wheels.append(
            WheelState(
                index=index,
                steer=steer,
                slip_angle=slip,
                normal_force=fz,
                mu=mu,
                lateral_force=fy,
                longitudinal_force=fx,
                brake_command=brake,
                saturated=saturated,
            )
        )

    drag = spec.drag_area * state.u * abs(state.u)
    fx_total -= drag

    ax = fx_total / spec.mass + state.v * state.r
    ay = fy_total / spec.mass - state.u * state.r

    return ForceBundle(
        fx_body=fx_total,
        fy_body=fy_total,
        mz_body=mz_total,
        ax=ax,
        ay=ay,
        wheels=tuple(wheels),
        drag=drag,
    )


def derivative(
    state: VehicleState,
    control: ControlInput,
    spec: VehicleSpec,
    tire: TireSpec,
    road: RoadSpec,
    ax_prev: float = 0.0,
    ay_prev: float = 0.0,
) -> tuple[VehicleState, ForceBundle]:
    """状态导数。返回 (导数状态, 受力明细)。"""
    bundle = wheel_forces(state, control, spec, tire, road, ax_prev, ay_prev)

    cos_psi, sin_psi = math.cos(state.yaw), math.sin(state.yaw)
    x_dot = state.u * cos_psi - state.v * sin_psi
    y_dot = state.u * sin_psi + state.v * cos_psi

    # 低速时把横摆角速度拉回零，避免静止时残留自转
    yaw_damping = 0.0
    if abs(state.u) < 0.6:
        yaw_damping = -1.5 * state.r

    u_dot = bundle.fx_body / spec.mass + state.v * state.r
    v_dot = bundle.fy_body / spec.mass - state.u * state.r
    r_dot = bundle.mz_body / spec.yaw_inertia + yaw_damping

    return VehicleState(x_dot, y_dot, state.r, u_dot, v_dot, r_dot), bundle


def rk4_step(
    state: VehicleState,
    control: ControlInput,
    spec: VehicleSpec,
    tire: TireSpec,
    road: RoadSpec,
    dt: float,
    ax_prev: float = 0.0,
    ay_prev: float = 0.0,
) -> tuple[VehicleState, ForceBundle]:
    """四阶 Runge-Kutta 推进一步。加速度反馈用步首的受力结果。"""

    def advance(base: VehicleState, slope: VehicleState, scale: float) -> VehicleState:
        return VehicleState(
            base.x + slope.x * scale,
            base.y + slope.y * scale,
            base.yaw + slope.yaw * scale,
            base.u + slope.u * scale,
            base.v + slope.v * scale,
            base.r + slope.r * scale,
        )

    k1, bundle = derivative(state, control, spec, tire, road, ax_prev, ay_prev)
    # 中间两步用步首的加速度反馈载荷转移，省一次自洽迭代
    mid = (bundle.ax, bundle.ay)
    k2, _ = derivative(advance(state, k1, dt / 2.0), control, spec, tire, road, *mid)
    k3, _ = derivative(advance(state, k2, dt / 2.0), control, spec, tire, road, *mid)
    k4, _ = derivative(advance(state, k3, dt), control, spec, tire, road, *mid)

    nxt = VehicleState(
        state.x + dt / 6.0 * (k1.x + 2 * k2.x + 2 * k3.x + k4.x),
        state.y + dt / 6.0 * (k1.y + 2 * k2.y + 2 * k3.y + k4.y),
        state.yaw + dt / 6.0 * (k1.yaw + 2 * k2.yaw + 2 * k3.yaw + k4.yaw),
        state.u + dt / 6.0 * (k1.u + 2 * k2.u + 2 * k3.u + k4.u),
        state.v + dt / 6.0 * (k1.v + 2 * k2.v + 2 * k3.v + k4.v),
        state.r + dt / 6.0 * (k1.r + 2 * k2.r + 2 * k3.r + k4.r),
    )
    # 归一化横摆角，避免长时间仿真后数值变大
    nxt.yaw = math.atan2(math.sin(nxt.yaw), math.cos(nxt.yaw))
    return nxt, bundle
