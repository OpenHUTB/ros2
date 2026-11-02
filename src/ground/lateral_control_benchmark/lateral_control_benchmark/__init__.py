#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""横向控制算法对比实验台（纯 Python，不依赖 ROS 与任何仿真器）。

快速开始::

    python3 main.py --mode bench            # 四场景 × 三控制器，输出指标表
    python3 main.py --mode bench --scenario arc
    python3 main.py --mode bench --model dynamic --speed 15
    python3 main.py --mode sweep --param stanley.k --values 0.6,1.2,2.0
    python3 main.py --mode ros              # 启动 ROS 1 节点
"""

from .controllers import LQR, BaseController, ControllerResult, PurePursuit, Stanley, build_controller  # noqa: F401
from .geometry import (  # noqa: F401
    Projection,
    ReferencePath,
    SCENARIOS,
    build_scenario,
    circular_arc,
    double_lane_change,
    from_csv,
    from_waypoints,
    sine_wave,
    straight_line,
    wrap_to_pi,
)
from .lqr_solver import solve_care  # noqa: F401
from .simulation import (  # noqa: F401
    Metrics,
    Simulator,
    SimulationConfig,
    SpeedProfile,
    Trace,
    evaluate,
    rank_controllers,
    track_speed,
)
from .vehicle_model import DynamicBicycle, KinematicBicycle, VehicleParams  # noqa: F401

__version__ = "1.0.0"

__all__ = [
    "PurePursuit", "Stanley", "LQR", "BaseController", "ControllerResult", "build_controller",
    "ReferencePath", "Projection", "build_scenario", "SCENARIOS",
    "straight_line", "circular_arc", "sine_wave", "double_lane_change",
    "from_waypoints", "from_csv", "wrap_to_pi",
    "VehicleParams", "KinematicBicycle", "DynamicBicycle",
    "Simulator", "SimulationConfig", "SpeedProfile", "Trace",
    "Metrics", "evaluate", "rank_controllers", "track_speed", "solve_care",
]
