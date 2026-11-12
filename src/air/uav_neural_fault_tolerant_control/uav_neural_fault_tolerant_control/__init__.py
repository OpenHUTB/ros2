"""无人机神经容错飞行控制：加权分配 + 神经网络残差。

这一层接在底座 ``uav_mujoco_sim`` 之上：几何控制器给出期望推力/力矩，
本包负责在桨故障情况下把它们映射成四桨推力。默认模式 ``neural``
先用加权最小范数解算出一个近似指令，再用一个小型 MLP 补上它与
精确箱约束解之间的差。

注意：本模块顶层不 import ``mujoco`` / ``torch``，没有仿真环境也能
``import uav_neural_fault_tolerant_control``（便于在纯 numpy 环境里读文档、
跑分配器数学测试）。
"""

from __future__ import annotations

from .allocation import (
    DESIRED_DIM,
    ROTOR_COUNT,
    build_effectiveness,
    clamp_thrusts,
    command_from_effective,
    effective_thrust,
    ls_allocation,
    naive_redistribute,
    solve_box_qp,
)
from .controller import MODES, FaultTolerantController
from .faults import DEFAULT_PLANS, FAULT_CN, FAULT_KINDS, FaultPlan
from .features import FEATURE_DIM, FEATURE_NAMES, apply_residual, build_features, build_target
from .trajectory import TRAJECTORIES, reference

__version__ = "1.0.0"

__all__ = [
    "__version__",
    # allocation
    "DESIRED_DIM",
    "ROTOR_COUNT",
    "build_effectiveness",
    "clamp_thrusts",
    "command_from_effective",
    "effective_thrust",
    "ls_allocation",
    "naive_redistribute",
    "solve_box_qp",
    # controller
    "MODES",
    "FaultTolerantController",
    # faults
    "DEFAULT_PLANS",
    "FAULT_CN",
    "FAULT_KINDS",
    "FaultPlan",
    # features
    "FEATURE_DIM",
    "FEATURE_NAMES",
    "apply_residual",
    "build_features",
    "build_target",
    # trajectory
    "TRAJECTORIES",
    "reference",
]
