# -*- coding: utf-8 -*-
"""humanoid_fall_protection 包。

对外只暴露三样东西：仿真封装、控制器、单次实验执行器。
"""

from .config import JOINT_NAMES, STRATEGIES, STRATEGY_LABELS  # noqa: F401
from .protection import FallController, FallDetector  # noqa: F401
from .runner import run_comparison, run_episode  # noqa: F401
from .sim_core import FallSim  # noqa: F401

__version__ = "1.0.0"
