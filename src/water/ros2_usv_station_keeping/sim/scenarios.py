"""六个可复现场景：从静水到大流、阵风、单侧失效。

设计原则：每个场景只改一个变量，这样"指标变差"能直接归因到那个变量上，而不是
一堆参数一起动、出问题说不清是谁的责任。
"""
from __future__ import annotations

from dataclasses import dataclass

from ros2_usv_station_keeping.vehicle import VehicleState

from .water_world import EnvironmentConfig


@dataclass(frozen=True)
class Scenario:
    """一个场景的完整定义。"""

    key: str
    title: str
    summary: str
    start: VehicleState
    target: tuple[float, float]
    env: EnvironmentConfig
    efficiency: tuple[float, float]
    duration: float = 180.0
    dt: float = 0.05

    @property
    def steps(self) -> int:
        return int(round(self.duration / self.dt))


_START = VehicleState(x=3.0, y=2.0, psi=2.4)


SCENARIOS: tuple[Scenario, ...] = (
    Scenario(
        key="calm",
        title="静水定点",
        summary="无流无风，验证基础收敛：应在 40 s 内进入 ±0.12 m 并能保持。",
        start=_START,
        target=(0.0, 0.0),
        env=EnvironmentConfig(seed=1),
        efficiency=(1.0, 1.0),
    ),
    Scenario(
        key="cross_current",
        title="横流 0.25 m/s",
        summary="正横流，船无法横向平移，只能靠 crab angle 斜着顶流；关掉补偿会持续偏出。",
        start=_START,
        target=(0.0, 0.0),
        env=EnvironmentConfig(current_x=0.0, current_y=0.25, seed=2),
        efficiency=(1.0, 1.0),
    ),
    Scenario(
        key="oblique_current",
        title="斜流 0.18/0.18 m/s",
        summary="流向与目标方位斜交，纵横向同时受扰，检验 crab 角在非正交方向上的效果。",
        start=_START,
        target=(0.0, 0.0),
        env=EnvironmentConfig(current_x=0.18, current_y=0.18, seed=3),
        efficiency=(1.0, 1.0),
    ),
    Scenario(
        key="gust",
        title="阵风 + 弱流",
        summary="叠加 OU 过程阵风，检验流估计低通是否把噪声抖成无效航向指令。",
        start=_START,
        target=(0.0, 0.0),
        env=EnvironmentConfig(current_x=0.20, current_y=0.12, gust_sigma=0.10, seed=4),
        efficiency=(1.0, 1.0),
    ),
    Scenario(
        key="degraded_thruster",
        title="单侧效率降至 55%",
        summary="左推进器效率下降，健康判定应进入 DEGRADED 并放宽保持死区，避免持续满推对抗。",
        start=_START,
        target=(0.0, 0.0),
        env=EnvironmentConfig(current_x=0.0, current_y=0.20, seed=5),
        efficiency=(0.55, 1.0),
    ),
    Scenario(
        key="failed_thruster",
        title="单侧完全失效",
        summary="左推进器失效，只剩单侧推力，定点能力必然下降；要求不失控、能上报并退到更大死区。",
        start=_START,
        target=(0.0, 0.0),
        env=EnvironmentConfig(current_x=0.0, current_y=0.18, seed=6),
        efficiency=(0.0, 1.0),
    ),
)


def get_scenario(key: str) -> Scenario:
    for scenario in SCENARIOS:
        if scenario.key == key:
            return scenario
    raise KeyError(f"未知场景：{key}")


def scenario_keys() -> tuple[str, ...]:
    return tuple(s.key for s in SCENARIOS)
