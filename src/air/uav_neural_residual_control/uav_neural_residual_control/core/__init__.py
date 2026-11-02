#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纯 Python 核心层：仅需 numpy，或 numpy + mujoco（plant / dataset / experiment）。

⚠️ 惰性导入：ROS 侧 Python 3.8 没有 mujoco，顶层导入 plant 会让节点一
import 就崩；disturbances / residual / rls / trajectories / controllers
只依赖 numpy，可以随意导入。
"""

__all__ = [
    "QuadParams", "QuadrotorState", "GeometricController", "mixer",
    "quat_to_mat", "mat_to_quat", "quat_to_euler", "RemotePlant",
    "GRAVITY", "DragModel", "WindModel", "PayloadModel", "PlantConfig",
    "PAYLOAD_LEVELS", "WIND_LEVELS", "make_plant",
    "ResidualPlant", "inject_payload", "nominal_params_of",
    "Trajectory", "Hover", "Circle", "Helix", "Figure8", "make_trajectory",
    "FEATURE_DIM", "build_features", "MLP", "ResidualNet",
    "RecursiveLeastSquares",
    "Gains", "Tracker", "PIDTracker", "GeometricTracker", "ResidualTracker",
    "ResidualRLSTracker", "TRACKERS", "make_tracker",
    "tracking_metrics", "residual_metrics", "format_table",
    "Dataset", "EpisodeCollector", "collect_dataset",
    "TrainConfig", "train_residual", "evaluate_residual",
    "RunRecord", "run_once", "run_grid", "save_results", "load_results",
]

#: 名字 → 子模块；带 mujoco 依赖的单列出来
_LAZY = {
    "ResidualPlant": "plant", "inject_payload": "plant",
    "nominal_params_of": "plant",
    "Dataset": "dataset", "EpisodeCollector": "dataset",
    "collect_dataset": "dataset",
    "RunRecord": "experiment", "run_once": "experiment",
    "run_grid": "experiment", "save_results": "experiment",
    "load_results": "experiment",
}

_NUMPY_ONLY = {
    "attitude": ["QuadParams", "QuadrotorState", "GeometricController", "mixer",
                 "quat_to_mat", "mat_to_quat", "quat_to_euler"],
    "remote_plant": ["RemotePlant"],
    "disturbances": ["GRAVITY", "DragModel", "WindModel", "PayloadModel",
                     "PlantConfig", "PAYLOAD_LEVELS", "WIND_LEVELS", "make_plant"],
    "trajectories": ["Trajectory", "Hover", "Circle", "Helix", "Figure8",
                     "make_trajectory"],
    "residual": ["FEATURE_DIM", "build_features", "MLP", "ResidualNet"],
    "rls": ["RecursiveLeastSquares"],
    "controllers": ["Gains", "Tracker", "PIDTracker", "GeometricTracker",
                    "ResidualTracker", "ResidualRLSTracker", "TRACKERS",
                    "make_tracker"],
    "metrics": ["tracking_metrics", "residual_metrics", "format_table"],
    "training": ["TrainConfig", "train_residual", "evaluate_residual"],
}
for _mod, _names in _NUMPY_ONLY.items():
    for _n in _names:
        _LAZY[_n] = _mod


def __getattr__(name):
    mod = _LAZY.get(name)
    if mod is None:
        raise AttributeError("module %r has no attribute %r" % (__name__, name))
    import importlib
    m = importlib.import_module("." + mod, __name__)
    val = getattr(m, name)
    globals()[name] = val
    return val


def __dir__():
    return sorted(set(list(globals()) + list(_LAZY)))
