#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跟踪性能指标。

统一在**稳态段**（去掉前 ``settle`` 秒）上统计：起飞段的暂态误差由初始条件
决定，与控制器好坏无关，算进去会把四个控制器的差距抹平。
"""
from __future__ import annotations

from typing import Dict, Optional

import numpy as np

__all__ = ["tracking_metrics", "residual_metrics", "format_table"]


def tracking_metrics(t: np.ndarray, pos_err: np.ndarray, settle: float = 3.0,
                     dt: float = 0.01) -> Dict[str, float]:
    """输入逐时刻的位置误差范数 (N,)，返回 RMSE / 最大值 / P95 / IAE。"""
    t = np.asarray(t, float)
    e = np.asarray(pos_err, float).reshape(-1)
    m = t >= settle
    if not np.any(m):
        m = np.ones_like(t, dtype=bool)
    e = e[m]
    return {
        "rmse": float(np.sqrt(np.mean(e ** 2))),
        "max": float(np.max(e)),
        "p95": float(np.percentile(e, 95)),
        "iae": float(np.sum(e) * dt),
        "mean": float(np.mean(e)),
    }


def residual_metrics(delta_true: np.ndarray, delta_hat: np.ndarray,
                     settle: float = 0.0) -> Dict[str, float]:
    """残差预测精度：机体系真值与预测的逐轴 RMSE 与决定系数。

    ``R²`` 用**零模型**（直接预测训练集均值）作基准：残差里载荷贡献的是一个
    近常值分量，只用 RMSE 看不出网络是否真的学到了速度相关部分。
    """
    y = np.asarray(delta_true, float)
    yh = np.asarray(delta_hat, float)
    if settle > 0.0:
        k = int(settle)
        y, yh = y[k:], yh[k:]
    if len(y) == 0:
        return {"res_rmse": float("nan"), "res_r2": float("nan")}
    err = yh - y
    ss_res = float(np.sum(err ** 2))
    ss_tot = float(np.sum((y - y.mean(axis=0)) ** 2))
    return {
        "res_rmse": float(np.sqrt(np.mean(err ** 2))),
        "res_rmse_xy": float(np.sqrt(np.mean(err[:, :2] ** 2))),
        "res_rmse_z": float(np.sqrt(np.mean(err[:, 2] ** 2))),
        "res_r2": float(1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else float("nan"),
        "res_bias": float(np.linalg.norm(err.mean(axis=0))),
    }


def format_table(rows, header, widths=None) -> str:
    """等宽纯文本表格，供终端与文档直接粘贴。"""
    widths = widths or [max(len(str(r[i])) for r in [header] + list(rows))
                        for i in range(len(header))]
    line = "  ".join(str(h).ljust(w) for h, w in zip(header, widths))
    out = [line, "-" * len(line)]
    for r in rows:
        out.append("  ".join(str(c).ljust(w) for c, w in zip(r, widths)))
    return "\n".join(out)


def summarize(records, keys=("rmse", "max", "iae")) -> Dict[str, Dict[str, float]]:
    """把多次运行的指标按控制器分组求均值。"""
    acc: Dict[str, Dict[str, list]] = {}
    for rec in records:
        d = acc.setdefault(rec["controller"], {k: [] for k in keys})
        for k in keys:
            d[k].append(rec["metrics"][k])
    return {c: {k: float(np.mean(v)) for k, v in d.items()} for c, d in acc.items()}
