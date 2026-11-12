"""闭环评估：跑一个场景，收集指标与时间序列。"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from ros2_usv_station_keeping.pilot import PilotConfig, StationKeepingPilot
from ros2_usv_station_keeping.vehicle import VehicleState

from .scenarios import Scenario, get_scenario
from .water_world import SimTrace, WaterWorld


@dataclass
class EpisodeMetrics:
    """一次仿真的量化结果。"""

    key: str
    rms_error: float = 0.0          # 尾段（最后 25%）距离 RMS
    max_error: float = 0.0
    final_error: float = 0.0
    settle_time: float = math.inf   # 首次进入并保持在 0.5 m 内的时间
    energy: float = 0.0             # ∫(|T_l| + |T_r|) dt，推力功的代理量
    saturation_ratio: float = 0.0
    mean_crab_deg: float = 0.0
    mean_abs_thrust: float = 0.0
    final_health: str = "NOMINAL"
    min_health_efficiency: float = 1.0
    completed: bool = True
    extra: dict = field(default_factory=dict)


def run_episode(
    scenario: Scenario | str,
    pilot_config: PilotConfig | None = None,
    noise_quality: float = 1.0,
    keep_trace: bool = True,
) -> tuple[EpisodeMetrics, SimTrace]:
    """跑一次闭环，返回指标与轨迹。"""
    sc = get_scenario(scenario) if isinstance(scenario, str) else scenario
    world = WaterWorld(
        state=VehicleState(
            x=sc.start.x,
            y=sc.start.y,
            psi=sc.start.psi,
            u=sc.start.u,
            v=sc.start.v,
            r=sc.start.r,
        ),
        env=sc.env,
        efficiency=sc.efficiency,
    )
    pilot = StationKeepingPilot(config=pilot_config)
    pilot.reset(sc.start.psi)

    trace = SimTrace(target=sc.target)
    dt = sc.dt
    distances: list[float] = []
    saturated_count = 0
    energy = 0.0

    control_dt = dt
    last_cmd_left = 0.0
    last_cmd_right = 0.0

    for step in range(sc.steps):
        t = step * dt
        obs = world.observe(noise_quality)
        observed = VehicleState(x=obs.x, y=obs.y, psi=obs.psi, u=obs.u, v=obs.v, r=obs.r)
        command = pilot.step(observed, sc.target, control_dt)
        last_cmd_left, last_cmd_right = command.left, command.right

        world.step(last_cmd_left, last_cmd_right, dt)

        s = world.state
        distance = math.hypot(s.x - sc.target[0], s.y - sc.target[1])
        distances.append(distance)
        energy += (abs(last_cmd_left) + abs(last_cmd_right)) * dt
        if command.snapshot.saturated:
            saturated_count += 1

        if keep_trace:
            trace.append(time=t)
            trace.append(x=s.x)
            trace.append(y=s.y)
            trace.append(psi=s.psi)
            trace.append(u=s.u)
            trace.append(v=s.v)
            trace.append(r=s.r)
            trace.append(distance=distance)
            trace.append(crab_deg=command.snapshot.crab_deg)
            trace.append(thrust_left=last_cmd_left)
            trace.append(thrust_right=last_cmd_right)
            trace.append(true_current=world.current)
            trace.append(est_current=pilot.estimated_current)
            trace.append(efficiency=command.snapshot.efficiency)
            trace.append(health=command.snapshot.health.value)
            trace.append(deadband=command.snapshot.in_deadband)

    metrics = _summarise(sc, distances, energy, saturated_count, trace, pilot)
    return metrics, trace


def _summarise(
    sc: Scenario,
    distances: list[float],
    energy: float,
    saturated_count: int,
    trace: SimTrace,
    pilot: StationKeepingPilot,
) -> EpisodeMetrics:
    n = len(distances)
    if n == 0:
        return EpisodeMetrics(key=sc.key, completed=False)

    tail_start = int(n * 0.75)
    tail = distances[tail_start:] or distances[-1:]
    rms = math.sqrt(sum(d * d for d in tail) / len(tail))

    # 首次进入 0.5 m 且此后不再离开
    settle = math.inf
    for i in range(n):
        if all(d <= 0.5 for d in distances[i:]):
            settle = i * sc.dt
            break

    crabs = [abs(c) for c in trace.crab_deg] if trace.crab_deg else [0.0]
    thrusts = [abs(a) + abs(b) for a, b in zip(trace.thrust_left, trace.thrust_right)] or [0.0]
    min_eff = min(trace.efficiency) if trace.efficiency else 1.0

    return EpisodeMetrics(
        key=sc.key,
        rms_error=rms,
        max_error=max(distances),
        final_error=distances[-1],
        settle_time=settle,
        energy=energy,
        saturation_ratio=saturated_count / n,
        mean_crab_deg=sum(crabs) / len(crabs),
        mean_abs_thrust=sum(thrusts) / len(thrusts),
        final_health=pilot.health.level.value,
        min_health_efficiency=min_eff,
        completed=True,
    )


def compare_crab(
    scenario: Scenario | str,
    noise_quality: float = 1.0,
) -> tuple[EpisodeMetrics, EpisodeMetrics]:
    """同一场景下对照：开启 / 关闭横漂补偿。"""
    sc = get_scenario(scenario) if isinstance(scenario, str) else scenario
    on, _ = run_episode(sc, PilotConfig(use_crab_compensation=True), noise_quality, keep_trace=False)
    off, _ = run_episode(sc, PilotConfig(use_crab_compensation=False), noise_quality, keep_trace=False)
    return on, off
