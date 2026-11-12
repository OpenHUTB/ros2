"""Reduced-order multirotor response model.

This is not a 6-DOF flight-dynamics model. It captures the dominant
weather-to-response chain needed by the risk demo: wind changes relative
airspeed, drag perturbs the vehicle, and the controller must spend tilt and
acceleration authority to compensate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from .types import Vec3, vadd, vclip, vnorm, vscale, vsub

GRAVITY = 9.80665


@dataclass(frozen=True)
class AircraftParams:
    mass_kg: float = 2.0
    air_density_kg_m3: float = 1.225
    drag_area_m2: float = 0.04
    max_accel_mps2: float = 5.0
    max_speed_mps: float = 7.0
    position_gain: float = 0.9
    velocity_gain: float = 1.2

    def __post_init__(self) -> None:
        for name in (
            "mass_kg",
            "air_density_kg_m3",
            "drag_area_m2",
            "max_accel_mps2",
            "max_speed_mps",
            "position_gain",
            "velocity_gain",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class AircraftState:
    position_enu: Vec3 = (0.0, 0.0, 30.0)
    velocity_enu: Vec3 = (0.0, 0.0, 0.0)
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    tilt_deg: float = 0.0
    control_saturation: float = 0.0
    acceleration_enu: Vec3 = (0.0, 0.0, 0.0)


class AircraftModel:
    def __init__(self, params: AircraftParams | None = None, state: AircraftState | None = None) -> None:
        self.params = params or AircraftParams()
        self.state = state or AircraftState()

    def clone(self) -> "AircraftModel":
        return AircraftModel(self.params, self.state)

    def step(self, dt_s: float, wind_enu: Vec3, target_enu: Vec3) -> AircraftState:
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("dt_s must be finite and positive")
        p = self.params
        state = self.state

        position_error = vsub(target_enu, state.position_enu)
        desired_velocity = vclip(vscale(position_error, p.position_gain), p.max_speed_mps)
        raw_accel = vscale(vsub(desired_velocity, state.velocity_enu), p.velocity_gain)
        command_accel = vclip(raw_accel, p.max_accel_mps2)

        relative_air = vsub(state.velocity_enu, wind_enu)
        relative_speed = vnorm(relative_air)
        drag_coeff = 0.5 * p.air_density_kg_m3 * p.drag_area_m2 / p.mass_kg
        drag_accel = vscale(relative_air, -drag_coeff * relative_speed)
        total_accel = vadd(command_accel, drag_accel)

        required_horizontal = (
            command_accel[0] - drag_accel[0],
            command_accel[1] - drag_accel[1],
            0.0,
        )
        horizontal_required = vnorm(required_horizontal)
        tilt = math.degrees(math.atan2(horizontal_required, GRAVITY))
        roll = math.degrees(math.atan2(required_horizontal[1], GRAVITY))
        pitch = math.degrees(math.atan2(required_horizontal[0], GRAVITY))
        saturation = horizontal_required / p.max_accel_mps2

        new_velocity = vadd(state.velocity_enu, vscale(total_accel, dt_s))
        new_position = vadd(
            state.position_enu,
            vscale(vadd(state.velocity_enu, new_velocity), 0.5 * dt_s),
        )
        if new_position[2] < 0.0:
            new_position = (new_position[0], new_position[1], 0.0)
            new_velocity = (new_velocity[0], new_velocity[1], max(0.0, new_velocity[2]))

        self.state = replace(
            state,
            position_enu=new_position,
            velocity_enu=new_velocity,
            roll_deg=roll,
            pitch_deg=pitch,
            tilt_deg=tilt,
            control_saturation=saturation,
            acceleration_enu=total_accel,
        )
        return self.state
