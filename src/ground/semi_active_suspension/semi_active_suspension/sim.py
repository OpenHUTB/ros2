"""仿真主循环与指标计算。"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from . import scenarios as scenario_lib
from . import suspension
from .params import DamperConfig, Disturbance, VehicleSpec, VehicleState

DT = 0.0005          # 积分步长，s
SAMPLE_DT = 0.002    # 采样步长，s


def static_state(spec: VehicleSpec) -> VehicleState:
    """静态平衡位形取作全零：方程写的是相对平衡点的增量，重力与预压已被消掉。"""
    return VehicleState()


@dataclass
class Frame:
    t: float
    state: VehicleState
    damping: tuple
    tire_load: tuple
    accel: float          # 车身垂向加速度，m/s^2

    @property
    def pitch_deg(self) -> float:
        return self.state.pitch * 180.0 / math.pi

    @property
    def roll_deg(self) -> float:
        return self.state.roll * 180.0 / math.pi


@dataclass
class SimResult:
    scenario: object
    semi_active: bool
    frames: list = field(default_factory=list)
    metrics: dict = field(default_factory=dict)

    def times(self) -> list:
        return [f.t for f in self.frames]

    def series(self, key: str) -> list:
        getters = {
            "accel": lambda f: f.accel,
            "pitch": lambda f: f.pitch_deg,
            "roll": lambda f: f.roll_deg,
            "damper": lambda f: sum(f.damping) / 4.0,
            "tire_front": lambda f: f.tire_load[0] + f.tire_load[1],
            "tire_rear": lambda f: f.tire_load[2] + f.tire_load[3],
            "tire_min": lambda f: min(f.tire_load),
            "deflection_front": lambda f: (f.state.z_u[0] - f.state.z_s
                                           + 1.15 * f.state.pitch),
        }
        getter = getters[key]
        return [getter(f) for f in self.frames]


def _disturbance(scenario, spec, t: float) -> Disturbance:
    road, road_rate = scenario_lib.road_inputs(scenario, spec, t)
    return Disturbance(road=road, road_rate=road_rate,
                       accel_x=scenario.longitudinal(t),
                       accel_y=scenario.lateral(t))


def _rms(values) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def run(scenario, spec: VehicleSpec | None = None,
        semi_active: bool = True, damper: DamperConfig | None = None) -> SimResult:
    spec = spec or VehicleSpec()
    damper = damper or DamperConfig()
    state = static_state(spec)
    result = SimResult(scenario=scenario, semi_active=semi_active)

    steps = int(round(scenario.duration / DT))
    sample_every = max(1, int(round(SAMPLE_DT / DT)))

    accels = []
    pitches = []
    rolls = []
    loads = []
    current = suspension.passive_damping(spec)
    alpha = min(1.0, DT / max(1e-6, damper.valve_tau))
    for step in range(steps + 1):
        t = step * DT
        dist = _disturbance(scenario, spec, t)
        if semi_active:
            desired = suspension.semi_active_damping(state, spec, damper)
            # 阻尼阀不是瞬间到位：一阶跟随，避免控制器每步在 min/max 之间硬切
            current = tuple(current[i] + (desired[i] - current[i]) * alpha
                            for i in range(4))
            damping = current
        else:
            damping = suspension.passive_damping(spec)
        accel = suspension.derivative(state, dist, spec, damping)[1]

        if step % sample_every == 0:
            frame = Frame(t=t, state=state.copy(), damping=damping,
                          tire_load=suspension.tire_load(state, dist, spec),
                          accel=accel)
            result.frames.append(frame)
            accels.append(accel)
            pitches.append(frame.pitch_deg)
            rolls.append(frame.roll_deg)
            loads.append(min(frame.tire_load))

        state = suspension.rk4_step(state, dist, spec, damping, DT)

    result.metrics = {
        "accel_rms": _rms(accels),
        "accel_peak": max(abs(a) for a in accels),
        "pitch_deg": max(abs(p) for p in pitches),
        "roll_deg": max(abs(r) for r in rolls),
        "load_min": min(loads),
        "load_rms": _rms([l - spec.static_load(0) for l in loads]),
        "travel_max": max(abs(f.state.z_u[0] - f.state.z_s) for f in result.frames),
        "damper_mean": (sum(sum(f.damping) / 4.0 for f in result.frames)
                        / len(result.frames)),
    }
    return result


def run_pair(scenario, spec: VehicleSpec | None = None,
             damper: DamperConfig | None = None):
    """同一工况跑被动与半主动两遍，便于画对照图。"""
    passive = run(scenario, spec, semi_active=False, damper=damper)
    active = run(scenario, spec, semi_active=True, damper=damper)
    return passive, active
