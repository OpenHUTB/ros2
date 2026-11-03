"""制动器温升与热衰退评估：集总热模型、摩擦衰退曲线与两个工况。

模块可在没有 ROS 2 的环境下单独运行（``python main.py demo``），ROS 2 节点只把
同一套计算接到话题上，见 ``ros_node.py``。
"""

from .disc import (
    BrakeDisc,
    BrakeDiscSpec,
    FrictionFadeCurve,
    STEFAN_BOLTZMANN,
)
from .scenario import (
    DESCENT_DURATION_S,
    DESCENT_GRADE,
    DESCENT_SPEEDS_KMH,
    STOP_COOLDOWN_S,
    STOP_CYCLES,
    STOP_SPEED_KMH,
    DemoBundle,
    DescentRun,
    StopCyclesRun,
    build_demo,
    kmh_to_mps,
    simulate_descent,
    simulate_stop_cycles,
    summary,
)
from .vehicle import (
    BrakingSpec,
    deceleration_mps2,
    per_disc_power_w,
    stop_energy_j,
    stopping_distance_m,
)

__all__ = [
    "BrakeDisc",
    "BrakeDiscSpec",
    "BrakingSpec",
    "DESCENT_DURATION_S",
    "DESCENT_GRADE",
    "DESCENT_SPEEDS_KMH",
    "DemoBundle",
    "DescentRun",
    "FrictionFadeCurve",
    "STEFAN_BOLTZMANN",
    "STOP_COOLDOWN_S",
    "STOP_CYCLES",
    "STOP_SPEED_KMH",
    "StopCyclesRun",
    "build_demo",
    "deceleration_mps2",
    "kmh_to_mps",
    "per_disc_power_w",
    "simulate_descent",
    "simulate_stop_cycles",
    "stop_energy_j",
    "stopping_distance_m",
    "summary",
]
