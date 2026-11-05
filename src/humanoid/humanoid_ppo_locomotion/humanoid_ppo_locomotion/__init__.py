"""G1 人形机器人 PPO 行走控制模块。

控制架构：解析平衡基元（:class:`ComBalance`）+ PPO 残差策略。
"""

from .balance import ComBalance
from .sim_core import G1Sim, SimConfig, NOMINAL_LEG_POSE, LEG_JOINTS, UPPER_JOINTS

__version__ = "0.2.0"

__all__ = [
    "ComBalance",
    "G1Sim",
    "SimConfig",
    "NOMINAL_LEG_POSE",
    "LEG_JOINTS",
    "UPPER_JOINTS",
]
