# -*- coding: utf-8 -*-
"""把重定时结果排成可直接贴进文档的 Markdown 表格。"""

from typing import Dict, List

import numpy as np

from .core import Limits, Trajectory, max_time_scale, peak_jerk, retime, utilization


def summarize(traj: Trajectory, limits: Limits, names: List[str]) -> str:
    r = max_time_scale(traj, limits)
    lines = [
        "| 指标 | 数值 |",
        "|---|---|",
        "| 关节数 | %d |" % traj.n_joints,
        "| 原始时长 | %.3f s |" % r["duration_before"],
        "| 最大时间缩放 α | **%.3f** |" % r["alpha"],
        "| 限制来源 | %s（关节 `%s`） |"
        % ("速度上限" if r["limited_by"] == "speed" else "加速度上限",
           names[r["joint"]]),
        "| 重定时后时长 | **%.3f s** |" % r["duration_after"],
        "| 压缩比 | %.1f%% |" % (100.0 * (1.0 - r["duration_after"] / r["duration_before"])),
    ]
    return "\n".join(lines)


def utilization_table(traj: Trajectory, limits: Limits, names: List[str]) -> str:
    """重定时后每个关节的占用率——1.00 就是"贴着上限跑"的那个关节。"""
    r = max_time_scale(traj, limits)
    fast = retime(traj, r["alpha"])
    u = utilization(fast, limits)
    lines = ["| 关节 | 峰值速度 (rad/s) | 速度占用 | 峰值加速度 (rad/s²) | 加速度占用 |",
             "|---|---|---|---|---|"]
    for j, name in enumerate(names):
        lines.append("| `%s` | %.2f | %.2f | %.2f | %.2f |"
                     % (name,
                        float(np.abs(fast.dq[j]).max()),
                        u["speed"][j],
                        float(np.abs(fast.ddq[j]).max()),
                        u["accel"][j]))
    return "\n".join(lines)


def jerk_table(traj: Trajectory, limits: Limits) -> str:
    r = max_time_scale(traj, limits)
    fast = retime(traj, r["alpha"])
    lines = ["| 指标 | 重定时前 | 重定时后 | 倍数 |", "|---|---|---|---|"]
    for label, fn in (("峰值加加速度 (rad/s³)", peak_jerk),
                      ("峰值速度 (rad/s)", lambda t: float(np.abs(t.dq).max())),
                      ("峰值加速度 (rad/s²)", lambda t: float(np.abs(t.ddq).max()))):
        a, b = fn(traj), fn(fast)
        lines.append("| %s | %.2f | %.2f | %.2f× |" % (label, a, b, b / a if a else 0.0))
    return "\n".join(lines)


def json_summary(traj: Trajectory, limits: Limits, names: List[str]) -> Dict:
    r = max_time_scale(traj, limits)
    fast = retime(traj, r["alpha"])
    return {
        "joints": names,
        "alpha": r["alpha"],
        "limited_by": r["limited_by"],
        "joint": r["joint"],
        "duration_before": r["duration_before"],
        "duration_after": r["duration_after"],
        "utilization": utilization(fast, limits),
        "peak_jerk_before": peak_jerk(traj),
        "peak_jerk_after": peak_jerk(fast),
    }
