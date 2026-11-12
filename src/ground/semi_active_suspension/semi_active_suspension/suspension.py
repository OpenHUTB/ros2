"""7 自由度整车垂向动力学与半主动阻尼的天棚/地棚控制。

自由度：车身垂向 z、俯仰 theta、侧倾 phi，加上四个非簧载质量的垂向位移。
轮胎只考虑垂向刚度与很小的阻尼，忽略轮胎包络效应。
"""
from __future__ import annotations

import math

from .params import (CORNERS, DamperConfig, Disturbance, VehicleSpec,
                     VehicleState)


def body_velocities(state: VehicleState, spec: VehicleSpec) -> tuple:
    """四个悬架安装点的车身垂向速度，m/s。"""
    return tuple(state.v_s - spec.axle_position(i) * state.pitch_rate
                 + spec.lateral_position(i) * state.roll_rate
                 for i in range(4))


def spring_forces(state: VehicleState, spec: VehicleSpec,
                  damping: tuple) -> tuple:
    """四个角上的悬架力，含弹簧与减振器，压缩为正。"""
    body_v = body_velocities(state, spec)
    forces = []
    for i in range(4):
        travel = state.z_u[i] - (state.z_s - spec.axle_position(i) * state.pitch
                                 + spec.lateral_position(i) * state.roll)
        rate = state.v_u[i] - body_v[i]
        forces.append(spec.spring_rate(i) * travel + damping[i] * rate)
    return tuple(forces)


def semi_active_damping(state: VehicleState, spec: VehicleSpec,
                        cfg: DamperConfig) -> tuple:
    """按天棚 + 地棚混合律算四个减振器的当前阻尼系数。

    半主动减振器只能耗能，不能出力，所以当车身速度与相对速度方向不一致时，
    想要的方向根本产生不了，只能退到最小阻尼。
    """
    body_v = body_velocities(state, spec)
    values = []
    for i in range(4):
        relative = state.v_u[i] - body_v[i]
        if abs(relative) < 1e-4:
            values.append(cfg.clamp_low(spec))
            continue
        desired = 0.0
        # 车身速度与相对速度反向时，减振器出力方向恰好能顶住车身，用天棚律
        if body_v[i] * relative < 0.0:
            desired = max(desired,
                          cfg.skyhook_gain * abs(body_v[i]) / abs(relative))
        # 车轮速度与相对速度同向时，加大阻尼能压住轮胎动载，用地棚律
        if state.v_u[i] * relative > 0.0:
            desired = max(desired,
                          cfg.groundhook_gain * abs(state.v_u[i]) / abs(relative))
        values.append(min(cfg.clamp_high(spec),
                          max(cfg.clamp_low(spec), desired)))
    return tuple(values)


def passive_damping(spec: VehicleSpec) -> tuple:
    return tuple(spec.passive_damping(i) for i in range(4))


def dynamic_tire_load(state: VehicleState, dist: Disturbance,
                      spec: VehicleSpec, corner: int) -> float:
    """相对静态平衡的轮胎垂向力增量，N。

    轮胎只能受压，所以先把总量夹到非负再减掉静态轮荷；离地时返回
    ``-static_load``，相当于车轮只受重力，不会出现"地面把车轮往下拽"。
    """
    static = spec.static_load(corner)
    total = (static
             + spec.tire_stiffness * (dist.road[corner] - state.z_u[corner])
             + spec.tire_damping * (dist.road_rate[corner] - state.v_u[corner]))
    return max(0.0, total) - static


def derivative(state: VehicleState, dist: Disturbance, spec: VehicleSpec,
               damping: tuple):
    """返回状态导数 (dz_s, dv_s, d_pitch, d_pitch_rate, d_roll, d_roll_rate, dz_u, dv_u)。"""
    forces = spring_forces(state, spec, damping)

    heave = sum(forces)
    pitch_moment = sum(-spec.axle_position(i) * forces[i] for i in range(4))
    roll_moment = sum(spec.lateral_position(i) * forces[i] for i in range(4))

    # 纵向减速时惯性力在质心高度上产生点头力矩；侧向加速度经侧倾力臂产生侧倾力矩
    pitch_moment += spec.sprung_mass * (-dist.accel_x) * spec.cg_height
    roll_moment += (spec.sprung_mass * dist.accel_y
                    * (spec.cg_height - spec.roll_center_height))

    accel_s = heave / spec.sprung_mass
    pitch_acc = pitch_moment / spec.inertia_pitch
    roll_acc = roll_moment / spec.inertia_roll

    wheel_acc = []
    for i in range(4):
        wheel_acc.append(
            (-forces[i] + dynamic_tire_load(state, dist, spec, i))
            / spec.unsprung_mass)

    return (state.v_s, accel_s, state.pitch_rate, pitch_acc, state.roll_rate,
            roll_acc, tuple(state.v_u), tuple(wheel_acc))


def advance(state: VehicleState, deriv, dt: float) -> VehicleState:
    """沿一次导数推进 dt（RK4 的中间步骤用）。"""
    d_z_s, d_v_s, _, d_pitch_rate, _, d_roll_rate, dz_u, dv_u = deriv
    return VehicleState(
        z_s=state.z_s + d_z_s * dt,
        v_s=state.v_s + d_v_s * dt,
        pitch=state.pitch + state.pitch_rate * dt,
        pitch_rate=state.pitch_rate + d_pitch_rate * dt,
        roll=state.roll + state.roll_rate * dt,
        roll_rate=state.roll_rate + d_roll_rate * dt,
        z_u=tuple(state.z_u[i] + dz_u[i] * dt for i in range(4)),
        v_u=tuple(state.v_u[i] + dv_u[i] * dt for i in range(4)),
    )


def rk4_step(state: VehicleState, dist: Disturbance, spec: VehicleSpec,
             damping: tuple, dt: float) -> VehicleState:
    """四阶龙格库塔推进一步。阻尼在一个步长内保持不变。"""
    k1 = derivative(state, dist, spec, damping)
    k2 = derivative(advance(state, k1, dt / 2.0), dist, spec, damping)
    k3 = derivative(advance(state, k2, dt / 2.0), dist, spec, damping)
    k4 = derivative(advance(state, k3, dt), dist, spec, damping)

    z_s = state.z_s + dt / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
    v_s = state.v_s + dt / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
    pitch = state.pitch + dt / 6.0 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2])
    pitch_rate = (state.pitch_rate + dt / 6.0
                  * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]))
    roll = state.roll + dt / 6.0 * (k1[4] + 2 * k2[4] + 2 * k3[4] + k4[4])
    roll_rate = (state.roll_rate + dt / 6.0
                 * (k1[5] + 2 * k2[5] + 2 * k3[5] + k4[5]))
    z_u = tuple(state.z_u[i] + dt / 6.0
                * (k1[6][i] + 2 * k2[6][i] + 2 * k3[6][i] + k4[6][i])
                for i in range(4))
    v_u = tuple(state.v_u[i] + dt / 6.0
                * (k1[7][i] + 2 * k2[7][i] + 2 * k3[7][i] + k4[7][i])
                for i in range(4))
    return VehicleState(z_s, v_s, pitch, pitch_rate, roll, roll_rate, z_u, v_u)


def tire_load(state: VehicleState, dist: Disturbance, spec: VehicleSpec) -> tuple:
    """四个轮胎的垂向载荷，N。含静态轮荷。"""
    return tuple(spec.static_load(i)
                 + dynamic_tire_load(state, dist, spec, i) for i in range(4))


def corner_names() -> tuple:
    return CORNERS
