"""仿真场景：标称过渡、粗糙倾角调度、高原、推力降级、阵风。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

from uav_tiltrotor_transition_corridor import (
    ControllerConfig,
    PilotConfig,
    TiltrotorSim,
    VehicleSpec,
)
from uav_tiltrotor_transition_corridor.world import SimResult

__all__ = ["Scenario", "SCENARIOS", "build", "run"]


@dataclass(frozen=True)
class Scenario:
    key: str
    title: str
    description: str
    hover_alt: float = 30.0
    corridor_alt: float = 0.0
    cfg: ControllerConfig = field(default_factory=ControllerConfig)
    pilot: PilotConfig = field(default_factory=PilotConfig)
    duration: float = 130.0
    wind: Callable[[float, float], tuple[float, float]] | None = None
    thrust_scale: Callable[[float], float] | None = None


def _gust(t: float, h: float) -> tuple[float, float]:
    """25–35 s 的 5 m/s 顶风（空气向 -x 运动 → ``u_r = u - wx`` 增大）。"""
    if 25.0 <= t <= 35.0:
        return -5.0, 0.0
    return 0.0, 0.0


def _headwind(t: float, h: float) -> tuple[float, float]:
    return -3.0, 0.0


def _degrade(t: float) -> float:
    """40 s 后总推力下降 15%（桨叶损伤 / 电机过温）。"""
    return 0.85 if t >= 40.0 else 1.0


SCENARIOS: dict[str, Scenario] = {
    "nominal": Scenario(
        key="nominal",
        title="标称过渡",
        description="30 m 高度悬停 → 加速到 20 m/s 巡航 → 减速回悬停，走廊保护开启。",
        duration=130.0,
    ),
    "fixed_tilt_no_guard": Scenario(
        key="fixed_tilt_no_guard",
        title="粗糙调度（无走廊保护）",
        description="高原机场（4000 m）上强行使用固定 15° 倾角，且不做走廊保护。",
        hover_alt=4030.0,
        corridor_alt=4000.0,
        cfg=ControllerConfig(tilt_schedule="fixed:15", corridor_guard=False),
        pilot=PilotConfig(hover_alt=4030.0),
        duration=50.0,
    ),
    "fixed_tilt_guard": Scenario(
        key="fixed_tilt_guard",
        title="粗糙调度（有走廊保护）",
        description="同一条 15° 调度曲线，但倾转角被走廊下边界兜住。",
        hover_alt=4030.0,
        corridor_alt=4000.0,
        cfg=ControllerConfig(tilt_schedule="fixed:15", corridor_guard=True),
        pilot=PilotConfig(hover_alt=4030.0),
        duration=50.0,
    ),
    "high_altitude": Scenario(
        key="high_altitude",
        title="高原过渡",
        description="4000 m 机场，走廊表按当地密度重算，自由分配。",
        hover_alt=4030.0,
        corridor_alt=4000.0,
        pilot=PilotConfig(hover_alt=4030.0),
        duration=130.0,
    ),
    "rotor_degraded": Scenario(
        key="rotor_degraded",
        title="推力降级",
        description="巡航中（40 s 后）总推力下降 15%，观察速度与高度如何退化。",
        thrust_scale=_degrade,
        duration=130.0,
    ),
    "headwind_gust": Scenario(
        key="headwind_gust",
        title="顶风阵风",
        description="25–35 s 遭遇 5 m/s 顶风，过渡段空速与地速分离。",
        wind=_gust,
        duration=130.0,
    ),
}


def build(key: str, spec: VehicleSpec | None = None) -> TiltrotorSim:
    sc = SCENARIOS[key]
    return TiltrotorSim(
        spec=spec,
        cfg=sc.cfg,
        pilot_cfg=sc.pilot,
        corridor_alt=sc.corridor_alt,
        wind=sc.wind,
        thrust_scale=sc.thrust_scale,
    )


def run(key: str, spec: VehicleSpec | None = None) -> SimResult:
    sc = SCENARIOS[key]
    sim = build(key, spec)
    return sim.run(sc.duration)
