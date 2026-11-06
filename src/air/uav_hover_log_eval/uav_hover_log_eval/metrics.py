"""校验定高日志，并从有限、单调的采样计算误差与扰动恢复指标。"""
import csv
import math
from pathlib import Path

import numpy as np


def load_log(path):
    """读取秒和米单位的日志；拒绝缺字段、非有限数及倒序时间。"""
    required = ('time_s', 'height_m', 'target_m')
    with Path(path).open(encoding='utf-8', newline='') as stream:
        reader = csv.DictReader(stream)
        if not set(required).issubset(reader.fieldnames or []):
            raise ValueError('日志必须包含 time_s、height_m、target_m')
        rows = [[float(row[key]) for key in required] for row in reader]
    data = np.asarray(rows, dtype=float)
    if len(rows) < 2 or not np.all(np.isfinite(data)):
        raise ValueError('日志至少需要两条有限数值记录')
    if np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError('时间戳必须严格递增')
    return data


def summarize(data, disturbance_end=3.0, tolerance=0.15, hold_s=0.5,
              disturbance_window=1.0, max_gap_s=0.05):
    """按采样点统计误差与扰动峰值，恢复判定需连续保持."""
    values = np.asarray(data, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3 or len(values) < 2:
        raise ValueError('需要至少两行、三列的定高数据')
    if not np.all(np.isfinite(values)) or np.any(np.diff(values[:, 0]) <= 0):
        raise ValueError('日志必须有限且时间严格递增')
    params = (disturbance_end, tolerance, hold_s, disturbance_window, max_gap_s)
    if (not all(math.isfinite(v) for v in params)
            or min(tolerance, hold_s, disturbance_window, max_gap_s) <= 0):
        raise ValueError('参数必须有限，容差、保持时间、扰动窗口和采样间隙必须为正')

    times, height, target = values.T
    error = height - target
    # 采样间隙阈值独立于输入日志，稀疏样本不能自行证明覆盖完整。
    gaps = np.diff(times)
    gap_limit = max_gap_s + 1e-9

    # 扰动窗口峰值偏差：抗扰能力的主指标，不受容差取值影响。
    window = (times >= disturbance_end - disturbance_window) & (times < disturbance_end)
    peak = float(np.max(np.abs(error[window]))) if window.any() else None
    # 标记扰动结束后的已观测样本是否超出容差带。
    left_band = bool(np.any(np.abs(error[times >= disturbance_end]) > tolerance))

    start, recovered = None, None
    for i, (now, residual) in enumerate(zip(times, error)):
        if now < disturbance_end:
            continue
        if abs(residual) > tolerance:
            start = None
            continue
        if start is None or (i and gaps[i - 1] > gap_limit):
            start = now
        if now - start >= hold_s - 1e-9:
            recovered = float(start - disturbance_end)
            break

    tail = times >= times[-1] - 1.0
    tail_times = times[tail]
    tail_covered = bool(
        len(tail_times) >= 2
        and tail_times[-1] - tail_times[0] >= 0.95
        and np.all(np.diff(tail_times) <= gap_limit))
    tail_error = np.abs(error[tail])
    complete = bool(
        0.0 <= times[0] <= 0.05 and times[-1] >= 7.95
        and np.all(gaps <= gap_limit))
    return {
        'samples': len(times),
        'duration_s': float(times[-1] - times[0]),
        'rmse_m': float(np.sqrt(np.mean(error ** 2))),
        'mae_m': float(np.mean(np.abs(error))),
        'max_error_m': float(np.max(np.abs(error))),
        'disturbance_peak_m': peak,
        'error_left_tolerance': left_band,
        'tail_covered': tail_covered,
        'tail_within_tolerance': bool(tail_covered and np.all(tail_error <= tolerance)),
        'tail_ratio': float(np.mean(tail_error <= tolerance)),
        'complete_8s_episode': complete,
        'recovery_s': recovered,
        'tolerance_m': tolerance,
        'hold_s': hold_s,
        'disturbance_end_s': disturbance_end,
        'disturbance_window_s': disturbance_window,
        'max_gap_s': max_gap_s,
    }
