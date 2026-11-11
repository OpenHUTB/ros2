"""humanoid_g1_compliance —— Unitree G1 (23 DOF) 全身柔顺控制。

LAC（Linear and Angular Compliance）预训练策略的 MuJoCo 复现与 ROS 2 封装：
给定 5 通道刚度指令，机器人在外力下按对应刚度柔顺让位并自动回位。
"""

__version__ = '1.0.0'

from .sim_core import G1Sim, JointState, resolve_model_dir          # noqa: F401
from .policy import (                                              # noqa: F401
    Command, LacPolicy, StartupFSM, STIFF_NAMES, DEFAULT_STIFFNESS_5,
)
from .compliance import (                                          # noqa: F401
    ComplianceMetrics, ComplianceRunner, Perturbation, RunLog,
)

__all__ = [
    'G1Sim', 'JointState', 'resolve_model_dir',
    'Command', 'LacPolicy', 'StartupFSM', 'STIFF_NAMES', 'DEFAULT_STIFFNESS_5',
    'ComplianceMetrics', 'ComplianceRunner', 'Perturbation', 'RunLog',
]
