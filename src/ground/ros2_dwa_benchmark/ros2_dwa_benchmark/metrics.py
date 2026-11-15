"""Metrics for the DWA stress benchmark.

Besides the navigation outcome (success, time, path, clearance) the benchmark
measures ride comfort from the applied velocity sequence: RMS linear/angular
acceleration and RMS jerk.  The simulator already applies the same acceleration
limits as the real controller, so these numbers describe the commands a robot
would actually receive.
"""

from __future__ import annotations

import numpy as np

from ros2_dwa_avoidance.sim import SimResult


def summarize(result: SimResult, dt: float = 0.1) -> dict:
    commands = np.asarray(result.commands, dtype=float) if result.commands else np.zeros((0, 2))
    v = commands[:, 0] if len(commands) else np.array([0.0])
    w = commands[:, 1] if len(commands) else np.array([0.0])

    if len(v) >= 2:
        acc = np.diff(v) / dt
        ang_acc = np.diff(w) / dt
        jerk = np.diff(acc) / dt if len(acc) >= 2 else np.array([0.0])
        linear_acc_rms = float(np.sqrt(np.mean(acc ** 2)))
        angular_acc_rms = float(np.sqrt(np.mean(ang_acc ** 2)))
        jerk_rms = float(np.sqrt(np.mean(jerk ** 2)))
        max_abs_acc = float(np.max(np.abs(acc)))
        smoothness = float(np.mean(np.abs(np.diff(v))) + np.mean(np.abs(np.diff(w))))
    else:
        linear_acc_rms = angular_acc_rms = jerk_rms = 0.0
        max_abs_acc = 0.0
        smoothness = 0.0

    return {
        "success": bool(result.success),
        "collision": bool(result.collision),
        "time_s": float(result.time),
        "path_length_m": float(result.path_length),
        "min_clearance_m": float(result.min_clearance),
        "final_distance_m": float(result.final_distance),
        "linear_acc_rms": round(linear_acc_rms, 4),
        "angular_acc_rms": round(angular_acc_rms, 4),
        "jerk_rms": round(jerk_rms, 4),
        "max_abs_acc": round(max_abs_acc, 4),
        "smoothness": round(smoothness, 5),
    }
