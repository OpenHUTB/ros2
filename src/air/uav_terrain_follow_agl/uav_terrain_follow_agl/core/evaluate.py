#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把一条架次的采样序列折成可比较的指标。

指标口径统一按「真值」算：``clearance = z_truth − h_truth(x)``，与控制器内部
看到的信息无关，避免「自己评自己」。
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, Sequence

import numpy as np


def _series(samples: Sequence, field: str) -> np.ndarray:
    return np.asarray([float(getattr(s, field)) for s in samples], dtype=float)


def clearance_series(samples: Sequence) -> np.ndarray:
    return _series(samples, "clearance")


def min_clearance(samples: Sequence) -> float:
    """整条航迹的最小离地间隙（米），负值表示侵入地形。"""
    c = clearance_series(samples)
    return float(np.min(c)) if c.size else float("nan")


def rms_agl_error(samples: Sequence, agl_set_m: float, warmup_s: float = 6.0) -> float:
    """加速段之后的离地高度 RMS 误差（米）。"""
    rows = [s for s in samples if s.t >= warmup_s]
    if not rows:
        return float("nan")
    err = np.asarray([float(s.agl_true) - float(agl_set_m) for s in rows])
    return float(np.sqrt(np.mean(err ** 2)))


def time_below(samples: Sequence, threshold_m: float) -> float:
    """离地间隙低于阈值的累计时长（秒）。"""
    if len(samples) < 2:
        return 0.0
    dt = float(samples[1].t - samples[0].t)
    flags = np.asarray([float(s.clearance) < float(threshold_m) for s in samples])
    return float(np.sum(flags) * dt)


def max_climb_rate(samples: Sequence, warmup_s: float = 6.0) -> float:
    """实际爬升率峰值（米/秒），取 |vz|。"""
    rows = [s for s in samples if s.t >= warmup_s]
    if not rows:
        return float("nan")
    return float(np.max(np.abs([float(s.vz) for s in rows])))


def summary(samples: Sequence, agl_set_m: float, safety_m: float,
            warmup_s: float = 6.0) -> Dict[str, float]:
    """一条架次的指标字典（可直接进 JSON）。"""
    if not samples:
        return {"samples": 0}
    c = clearance_series(samples)
    i_min = int(np.argmin(c))
    rows = [s for s in samples if s.t >= warmup_s] or list(samples)
    err = np.asarray([float(s.agl_true) - float(agl_set_m) for s in rows])
    first = next((s for s in samples if s.clearance < 0.0), None)
    return {
        "samples": int(len(samples)),
        "distance_m": float(samples[-1].x - samples[0].x),
        "duration_s": float(samples[-1].t - samples[0].t),
        "min_clearance_m": float(c[i_min]),
        "min_clearance_x_m": float(samples[i_min].x),
        "intrusion_m": float(max(0.0, -c[i_min])),
        "first_intrusion_x_m": None if first is None else float(first.x),
        "mean_clearance_m": float(np.mean(c)),
        "max_clearance_m": float(np.max(c)),
        "rms_agl_error_m": float(np.sqrt(np.mean(err ** 2))),
        "max_agl_error_m": float(np.max(np.abs(err))),
        "time_below_safety_s": time_below(samples, safety_m),
        "time_below_zero_s": time_below(samples, 0.0),
        "max_climb_rate_mps": max_climb_rate(samples, warmup_s),
        "crashed": bool(np.min(c) < 0.0),
    }


def aggregate(rows: Iterable[Dict[str, float]]) -> Dict[str, float]:
    """多架次取最坏值/均值：间隙取最小，误差取 RMS 均值。"""
    rows = list(rows)
    if not rows:
        return {}
    return {
        "runs": len(rows),
        "min_clearance_m": min(r["min_clearance_m"] for r in rows),
        "mean_clearance_m": float(np.mean([r["mean_clearance_m"] for r in rows])),
        "rms_agl_error_m": float(np.mean([r["rms_agl_error_m"] for r in rows])),
        "max_climb_rate_mps": max(r["max_climb_rate_mps"] for r in rows),
        "crashed_runs": int(sum(1 for r in rows if r["crashed"])),
    }


def format_table(rows: Sequence[Dict], columns: Sequence[str],
                 headers: Sequence[str] | None = None) -> str:
    """极简等宽表格（不引第三方依赖）。"""
    heads = list(headers or columns)
    cells = []
    for r in rows:
        line = []
        for col in columns:
            val = r.get(col, "")
            if isinstance(val, bool):
                line.append("是" if val else "否")
            elif isinstance(val, float):
                line.append("nan" if math.isnan(val) else f"{val:.2f}")
            else:
                line.append(str(val))
        cells.append(line)
    widths = [max(len(heads[i]), *(len(c[i]) for c in cells)) for i in range(len(heads))
              ] if cells else [len(h) for h in heads]
    out = ["  ".join(h.ljust(widths[i]) if i < len(widths) - 1 else h
                     for i, h in enumerate(heads))]
    out.append("  ".join("-" * w for w in widths))
    for c in cells:
        out.append("  ".join(v.ljust(widths[i]) if i < len(widths) - 1 else v
                             for i, v in enumerate(c)))
    return "\n".join(out)
