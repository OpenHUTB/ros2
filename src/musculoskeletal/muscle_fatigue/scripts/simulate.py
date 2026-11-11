# -*- coding: utf-8 -*-
"""肌肉疲劳仿真：持续收缩 vs 间歇收缩。

用 muscle_model 里的 Hill + 疲劳模型，在最优长度、零速度下跑两种激活协议：

    协议 A（持续）：激活度 a 恒为 0.8，连续 30 s
    协议 B（间歇）：a = 0.8 维持 2 s，再休息 2 s，循环 30 s

输出两条"归一化力 F/F_max"曲线，直观展示：持续收缩会掉到疲劳下界，
而间歇收缩因每个休息段恢复一部分疲劳，平均力更高（依赖占空比）。

只用标准库；折线图用 simple_plot 手写 PNG。
"""

from __future__ import annotations

import os

from muscle_model import Muscle, MuscleParams, FatigueParams
from simple_plot import plot_series

DT = 0.01          # 仿真步长 (s)
TOTAL = 30.0       # 仿真总时长 (s)
A_ON = 0.8         # 收缩时的激活度
DUTY_ON = 2.0      # 间歇协议：收缩段时长 (s)
DUTY_OFF = 2.0     # 间歇协议：休息段时长 (s)


def _activation(t, protocol):
    if protocol == "sustained":
        return A_ON
    # intermittent
    cycle = DUTY_ON + DUTY_OFF
    return A_ON if (t % cycle) < DUTY_ON else 0.0


def run(protocol, params=None, fatigue=None, total=TOTAL):
    """跑一种协议，返回 (ts, act, force_norm)。force_norm = F / F_max。"""
    m = Muscle(params or MuscleParams(), fatigue or FatigueParams())
    n = int(round(total / DT))
    ts, act, fnorm = [], [], []
    for i in range(n + 1):
        t = i * DT
        u = _activation(t, protocol)
        F = m.step(u, DT, l_n=1.0, v_n=0.0)
        ts.append(t)
        act.append(u)
        fnorm.append(F / m.params.f_max)
    return ts, act, fnorm


def build_series():
    ts_s, act_s, f_s = run("sustained")
    ts_i, act_i, f_i = run("intermittent")
    return [
        ("sustained F/Fmax", ts_s, f_s, (200, 40, 40)),
        ("intermittent F/Fmax", ts_i, f_i, (30, 90, 200)),
        ("intermittent activation", ts_i, act_i, (120, 120, 120)),
    ]


def main(out_dir=".", quick=False):
    os.makedirs(out_dir, exist_ok=True)
    total = 10.0 if quick else TOTAL
    series = build_series()
    path = os.path.join(out_dir, "fatigue_curves.png")
    plot_series(
        path,
        series,
        title="Muscle fatigue: sustained vs intermittent",
        width=900, height=540,
        margin=(60, 40, 30, 150),
        ylabel="F/Fmax or activation",
    )
    # 汇报关键数值
    _, _, f_s = run("sustained", total=total)
    _, _, f_i = run("intermittent", total=total)
    print("持续收缩末段 F/Fmax 均值: %.3f" % (sum(f_s[-500:]) / 500))
    print("间歇收缩全程 F/Fmax 均值: %.3f" % (sum(f_i) / len(f_i)))
    print("已生成: %s" % path)
    return path


if __name__ == "__main__":
    main(os.path.dirname(os.path.abspath(__file__)))
