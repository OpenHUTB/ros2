#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""评价指标。

只报"跑起来了"没有说服力，本模块用四个可复算的量描述控制品质：

* `settle_time`         进入并保持在判据带内的时刻（越小越快）
* `peak_overshoot`      超调量（相对目标幅值的百分比，越小越稳）
* `steady_state_error`  末段平均误差（反映积分项是否够用）
* `max_abs_deviation`   全程最大偏离（反映抗扰/起始冲击）

每个指标都写成**纯函数**（输入时间序列，输出标量），这样单元测试可以直接喂
人造序列验证定义，不需要跑仿真。
"""

import numpy as np


def _as_arrays(t, y):
    t = np.asarray(t, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()
    if t.size != y.size:
        raise ValueError(f"时间与信号长度不一致：{t.size} vs {y.size}")
    if t.size == 0:
        raise ValueError("空信号")
    return t, y


def settle_time(t, y, target, tol, hold_ratio=1.0):
    """进入判据带 |y-target| <= tol 并**一直保持**到结束的最早时刻。

    `hold_ratio=1.0` 表示"此后再也不能出带"；小于 1 时允许最后 hold_ratio 比例
    的时长内出带（抗扰场景里信号会再被推一下，需要放宽时用）。
    若全程未进入判据带，返回 `float('nan')`——nan 会让上层的均值/成功率统计
    明确地"算不出来"，而不是悄悄把失败当成 0 秒。
    """
    t, y = _as_arrays(t, y)
    inside = np.abs(y - float(target)) <= float(tol)
    if not np.any(inside):
        return float("nan")
    if hold_ratio >= 1.0:
        n_hold = 1
    else:
        n_hold = max(1, int(round(len(t) * float(hold_ratio))))
    # 从后往前扫：找到最后一个"出带"点，其后第一个点即稳定起点
    idx_outside = np.nonzero(~inside)[0]
    if idx_outside.size == 0:
        return float(t[0])
    last_out = int(idx_outside[-1])
    if last_out >= len(t) - n_hold:
        return float("nan")
    return float(t[last_out + 1])


def peak_overshoot(y, target, start_value=None):
    """超调百分比：相对"从 start_value 走到 target"这段行程的最大越过量。

    行程接近 0 时（目标≈起点）超调没有意义，返回 0.0 而不是 inf。
    """
    y = np.asarray(y, dtype=float).ravel()
    target = float(target)
    start = float(y[0] if start_value is None else start_value)
    travel = abs(target - start)
    if travel < 1e-9:
        return 0.0
    if target >= start:
        peak = float(np.max(y) - target)
    else:
        peak = float(target - np.min(y))
    return max(0.0, peak) / travel * 100.0


def steady_state_error(t, y, target, frac=0.10):
    """末段 `frac` 时长内的平均误差绝对值。"""
    t, y = _as_arrays(t, y)
    n = max(1, int(round(len(t) * float(frac))))
    return float(np.mean(np.abs(y[-n:] - float(target))))


def max_abs_deviation(y, target):
    y = np.asarray(y, dtype=float).ravel()
    return float(np.max(np.abs(y - float(target))))


def rms(y, target):
    y = np.asarray(y, dtype=float).ravel()
    return float(np.sqrt(np.mean((y - float(target)) ** 2)))


def summarize_series(t, y, target, tol, frac=0.10):
    """把上面几个指标打包成一个 dict（便于直接写 CSV）。"""
    return {
        "settle_time": settle_time(t, y, target, tol),
        "overshoot_pct": peak_overshoot(y, target),
        "steady_state_error": steady_state_error(t, y, target, frac),
        "max_abs_deviation": max_abs_deviation(y, target),
    }
