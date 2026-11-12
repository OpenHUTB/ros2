"""Explain closed-loop ATO energy use by physical work components."""

from __future__ import annotations

from dataclasses import asdict, dataclass

from .sim import SimResult
from .train import TrainSpec, grade_at

J_PER_KWH = 3.6e6


@dataclass(frozen=True)
class EnergyBreakdown:
    traction_work_kwh: float
    traction_limit_shortfall_kwh: float
    retarding_work_kwh: float
    net_inertial_work_kwh: float
    acceleration_work_kwh: float
    deceleration_work_kwh: float
    resistance_work_kwh: float
    grade_work_kwh: float
    endpoint_kinetic_energy_kwh: float

    def as_dict(self) -> dict:
        return {
            key: round(value, 6)
            for key, value in asdict(self).items()
        }


def energy_breakdown(result: SimResult,
                     spec: TrainSpec | None = None) -> EnergyBreakdown:
    """Integrate control-step work using the same v*dt convention as sim.run."""
    spec = spec or result.spec
    traction = shortfall = retarding = inertial = 0.0
    acceleration = deceleration = 0.0
    resistance = grade = 0.0
    for current, following in zip(result.frames, result.frames[1:]):
        distance = max(0.0, current.v * result.dt)
        if distance <= 0.0 or current.v <= 0.0:
            continue
        # frame.accel stores the actuator value applied on the following step.
        accel = following.accel
        inertial_force = spec.rotary_factor * spec.mass * accel
        resistance_force = spec.resistance(current.v)
        grade_force = spec.grade_force(
            grade_at(result.segments, current.x))
        net_force = inertial_force + resistance_force + grade_force
        available_force = spec.traction_force(current.v)
        traction_force = min(available_force, max(0.0, net_force))
        retarding_force = max(0.0, -net_force)
        shortfall_force = max(0.0, net_force - available_force)
        traction += traction_force * distance
        shortfall += shortfall_force * distance
        retarding += retarding_force * distance
        inertial += inertial_force * distance
        acceleration += max(0.0, inertial_force) * distance
        deceleration += max(0.0, -inertial_force) * distance
        resistance += resistance_force * distance
        grade += grade_force * distance
    final_v = result.frames[-1].v if result.frames else 0.0
    endpoint_kinetic = (
        0.5 * spec.rotary_factor * spec.mass * final_v * final_v
    )
    return EnergyBreakdown(
        traction_work_kwh=traction / J_PER_KWH,
        traction_limit_shortfall_kwh=shortfall / J_PER_KWH,
        retarding_work_kwh=retarding / J_PER_KWH,
        net_inertial_work_kwh=inertial / J_PER_KWH,
        acceleration_work_kwh=acceleration / J_PER_KWH,
        deceleration_work_kwh=deceleration / J_PER_KWH,
        resistance_work_kwh=resistance / J_PER_KWH,
        grade_work_kwh=grade / J_PER_KWH,
        endpoint_kinetic_energy_kwh=endpoint_kinetic / J_PER_KWH,
    )
