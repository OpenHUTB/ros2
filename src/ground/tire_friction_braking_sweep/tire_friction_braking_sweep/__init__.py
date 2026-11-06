"""轮胎附着系数与制动距离的实测扫掠、两区段建模与瓶颈判定。

``python main.py demo`` 不需要模拟器；``python main.py sim`` 会在 CARLA/HUTB 里跑真实试验。
"""

from .analyze import (
    BrakingPoint,
    SaturationFit,
    SweepDataset,
    classify_points,
    fit_saturation,
    summary,
)
from .model import (
    GRAVITY,
    BrakingSpec,
    bottleneck,
    braking_distance_m,
    friction_bound_decel_mps2,
    ideal_decel_mps2,
    saturated_decel_mps2,
)
from .sim import (
    DEFAULT_FRICTIONS,
    DEFAULT_VEHICLE,
    SimEnvironment,
    connect,
    run_sweep,
)

__all__ = [
    "BrakingPoint",
    "BrakingSpec",
    "DEFAULT_FRICTIONS",
    "DEFAULT_VEHICLE",
    "GRAVITY",
    "SaturationFit",
    "SimEnvironment",
    "SweepDataset",
    "bottleneck",
    "braking_distance_m",
    "classify_points",
    "connect",
    "fit_saturation",
    "friction_bound_decel_mps2",
    "ideal_decel_mps2",
    "run_sweep",
    "saturated_decel_mps2",
    "summary",
]
