#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stanley 前轴横向偏差跟踪模块（纯 Python 实现，可选 ROS 2 封装）。

快速开始::

    python3 main.py --mode demo            # 跑全部场景并导出效果图
    python3 main.py --mode sweep           # 扫描横向增益 k
    python3 test/test_stanley.py           # 离线单元测试（不依赖 ROS）
"""

from .controllers import (  # noqa: F401
    AdaptivePurePursuitController,
    BaseController,
    ControlCommand,
    LookaheadPIDController,
    StanleyController,
    build_controller,
)
from .geometry import ProjectionResult, ReferencePath, normalize, wrap_to_pi  # noqa: F401
from .reference import (  # noqa: F401
    SCENARIOS,
    build_scenario,
    circular_arc,
    double_lane_change,
    from_csv,
    from_waypoints,
    sine_wave,
    straight_line,
)
from .simulator import (  # noqa: F401
    Metrics,
    SimulationConfig,
    SimulationTrace,
    Simulator,
    SpeedProfile,
    evaluate,
)

__all__ = [
    "StanleyController",
    "AdaptivePurePursuitController",
    "LookaheadPIDController",
    "BaseController",
    "ControlCommand",
    "build_controller",
    "ReferencePath",
    "ProjectionResult",
    "wrap_to_pi",
    "normalize",
    "build_scenario",
    "straight_line",
    "circular_arc",
    "sine_wave",
    "double_lane_change",
    "from_waypoints",
    "from_csv",
    "SCENARIOS",
    "Simulator",
    "SimulationConfig",
    "SimulationTrace",
    "SpeedProfile",
    "Metrics",
    "evaluate",
]

__version__ = "1.0.0"
