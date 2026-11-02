"""校验定高日志，并从有限、单调的采样计算误差与恢复时间。"""
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


def summarize(data, disturbance_end=3.0, tolerance=0.15, hold_s=0.5):
    """按采样点计算 RMSE；恢复需连续保持，间隙过大不计作连续。"""
    values = np.asarray(data, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3 or len(values) < 2:
        raise ValueError('需要至少两行、三列的定高数据')
    if not np.all(np.isfinite(values)) or np.any(np.diff(values[:, 0]) <= 0):
        raise ValueError('日志必须有限且时间严格递增')
    if not all(math.isfinite(v) for v in (disturbance_end, tolerance, hold_s)) or tolerance <= 0 or hold_s <= 0:
        raise ValueError('恢复参数必须有限，容差和保持时间必须为正')
    times, height, target = values.T
    error = height - target
    median_dt = float(np.median(np.diff(times)))
    start, recovered = None, None
    for i, (now, residual) in enumerate(zip(times, error)):
        if now < disturbance_end:
            continue
        if abs(residual) > tolerance or (i and now - times[i-1] > 2.5 * median_dt):
            start = None
            continue
        if start is None:
            start = now
        if now - start >= hold_s - 1e-9:
            recovered = float(start - disturbance_end)
            break
    tail = times >= times[-1] - 1.0
    complete = times[0] <= 0.021 and times[-1] >= 7.999
    tail_covered = times[-1] - times[tail][0] >= 0.95
    return {'samples': len(times), 'duration_s': float(times[-1] - times[0]),
            'rmse_m': float(np.sqrt(np.mean(error ** 2))),
            'mae_m': float(np.mean(np.abs(error))),
            'max_error_m': float(np.max(np.abs(error))),
            'tail_within_tolerance': bool(tail_covered and np.all(np.abs(error[tail]) <= tolerance)),
            'complete_8s_episode': bool(complete),
            'recovery_s': recovered, 'tolerance_m': tolerance, 'hold_s': hold_s}
