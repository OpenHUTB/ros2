# -*- coding: utf-8 -*-
"""把悬停可行域结果排成 Markdown 表格。"""

from typing import List, Sequence

from .core import (Config, max_hover, max_hover_zero_yaw, nominal_mass_limit,
                   spin_rate_for)


def table_configs(configs: Sequence[Config]) -> str:
    """每种机型：正常 / 单桨失效（自由偏航 / 零偏航）的推力余量。"""
    lines = ["| 机型 | 桨数 | 正常 | 单桨失效（可自旋） | 单桨失效（零偏航） | 结论 |",
             "|---|---|---|---|---|---|"]
    for cfg in configs:
        base = max_hover(cfg)
        free = max_hover(cfg, {0})
        zero = max_hover_zero_yaw(cfg, {0})
        if zero["margin"] > 0.99 * free["margin"]:
            concl = "不转也能悬停"
        elif zero["margin"] <= 1e-6:
            concl = "**必须自旋**，否则 0%"
        else:
            concl = "自旋多 %.1f 个百分点" % (100 * (free["margin"] - zero["margin"]))
        lines.append("| %s | %d | %.1f%% | %.1f%% | %.1f%% | %s |"
                     % (cfg.name, cfg.n(), 100 * base["margin"],
                        100 * free["margin"], 100 * zero["margin"], concl))
    return "\n".join(lines)


def table_detail(cfg: Config) -> str:
    """某一机型逐桨失效的明细（含残余偏航力矩与估算自旋角速度）。"""
    lines = ["| 失效桨 | 总推力 (N) | 载重上限 (kg) | 余量 | 残余 Mz (N·m) | 自旋角速度 (rad/s) |",
             "|---|---|---|---|---|---|"]
    base = max_hover(cfg)["total_thrust"]
    for k in range(cfg.n()):
        r = max_hover(cfg, {k})
        omega = 0.0 if r["yaw_free"] else spin_rate_for(r["yaw_torque"])
        lines.append("| #%d | %.1f | %.2f | %.1f%% | %+.4f | %.1f |"
                     % (k, r["total_thrust"], r["mass_limit"],
                        100 * r["total_thrust"] / base, r["yaw_torque"], omega))
    return "\n".join(lines)


def table_thrusts(cfg: Config, failed=(0,)) -> str:
    """最优解下每个桨的推力分配（失效桨应为 0）。"""
    r = max_hover(cfg, set(failed))
    lines = ["| 桨 | 推力 (N) | 占比 |", "|---|---|---|"]
    for i, f in enumerate(r["thrusts"]):
        tag = "（失效）" if i in failed else ""
        lines.append("| #%d%s | %.2f | %.0f%% |"
                     % (i, tag, f, 100 * f / cfg.thrust_max))
    return "\n".join(lines)


def json_summary(configs: Sequence[Config]) -> List[dict]:
    out = []
    for cfg in configs:
        free = max_hover(cfg, {0})
        zero = max_hover_zero_yaw(cfg, {0})
        out.append({
            "config": cfg.name, "rotors": cfg.n(),
            "nominal_mass_limit_kg": nominal_mass_limit(cfg),
            "failure_free_yaw_margin": free["margin"],
            "failure_zero_yaw_margin": zero["margin"],
            "failure_mass_limit_kg": free["mass_limit"],
            "yaw_torque": free["yaw_torque"],
            "yaw_free": free["yaw_free"],
        })
    return out
