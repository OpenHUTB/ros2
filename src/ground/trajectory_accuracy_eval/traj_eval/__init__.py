#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轨迹 / 里程计精度评估工具（纯 Python，不依赖 ROS 与任何仿真器）。

快速开始::

    python3 main.py --mode demo            # 生成一条带误差的里程计轨迹并评估
    python3 main.py --mode eval --gt gt.csv --est est.csv
    python3 main.py --mode sweep           # 扫描轮半径尺度误差的影响
    python3 main.py --mode ros             # 启动 ROS 1 在线评估

指标定义见 :mod:`traj_eval.metrics`，公式与使用说明见模块内 ``docs/ground/trajectory_accuracy_eval.md``。
"""

from .io import load_trajectory, save_trajectory  # noqa: F401
from .metrics import (  # noqa: F401
    ATEResult,
    RPEResult,
    ate,
    drift_percent,
    evaluate_all,
    rpe,
)
from .se2 import (  # noqa: F401
    Alignment,
    Pose,
    align_umeyama,
    compose,
    inverse,
    path_length,
    positions,
    transform_poses,
    wrap_angle,
    yaws,
)
from .simulate import (  # noqa: F401
    KINDS,
    OdomErrorModel,
    default_error_models,
    make_ground_truth,
    simulate_odometry,
)

__version__ = "1.0.0"

__all__ = [
    "Pose", "Alignment", "wrap_angle", "compose", "inverse",
    "positions", "yaws", "path_length", "align_umeyama", "transform_poses",
    "ate", "rpe", "drift_percent", "evaluate_all", "ATEResult", "RPEResult",
    "make_ground_truth", "simulate_odometry", "OdomErrorModel",
    "default_error_models", "KINDS",
    "save_trajectory", "load_trajectory",
]
