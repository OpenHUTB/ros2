# -*- coding: utf-8 -*-
"""把仿真结果排成可直接贴进文档的 Markdown 表格。"""

from .core import (VehicleSpec, compare_with_ideal, ideal_stopping_distance,
                   simulate_braking, simulate_braking_ideal, simulate_lateral,
                   sweep_bias, sweep_payload_position)


def table_case(spec: VehicleSpec) -> str:
    r = simulate_braking(spec)
    c = compare_with_ideal(spec)
    lines = [
        "| 指标 | 数值 |",
        "|---|---|",
        "| 初速 | %.1f km/h |" % (spec.v0 * 3.6),
        "| 总质量 | %.0f kg |" % spec.total_mass(),
        "| 合成质心（距前轴） | %.2f m |" % spec.com_position(),
        "| 制动距离 | **%.2f m** |" % r["distance_m"],
        "| 理想 EBD 距离（含风阻，按动态轴荷） | %.2f m |"
        % simulate_braking_ideal(spec)["distance_m"],
        "| 与理想差距 | %+.1f%%（%+.2f m） |" % (c["gap_pct"], c["gap_m"]),
        "| 峰值减速度 | %.2f m/s² |" % r["peak_decel"],
        "| 先抱死轴 | %s |" % r["first_lock"],
        "| 后轴抬轮 | %s |" % ("是" if r["rear_lift"] else "否"),
    ]
    return "\n".join(lines)


def table_payload_sweep(spec: VehicleSpec, payload: float = 300.0) -> str:
    rows = sweep_payload_position(spec, payload=payload)
    base = simulate_braking(spec)["distance_m"]
    lines = ["| 载重位置（距前轴） | 合成质心 | 前轴载荷 | 后轴载荷 | 制动距离 | 相对空载 | 先抱死 |",
             "|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append("| %.2f m | %.2f m | %.0f N | %.0f N | %.2f m | %+.1f%% | %s |"
                     % (r["payload_x"], r["com_x"], r["fz_front"], r["fz_rear"],
                        r["distance_m"], (r["distance_m"] - base) / base * 100.0,
                        r["first_lock"]))
    return "\n".join(lines)


def table_bias_sweep(spec: VehicleSpec, payload: float = 300.0,
                     payload_x: float = 2.4) -> str:
    rows = sweep_bias(spec, payload=payload, payload_x=payload_x)
    best = min(rows, key=lambda r: r["distance_m"])
    lines = ["| 前轴制动力占比 β | 制动距离 | 先抱死 |", "|---|---|---|"]
    for r in rows:
        mark = " ← 最优" if r is best else ""
        lines.append("| %.2f | %.2f m%s | %s |"
                     % (r["brake_bias"], r["distance_m"], mark, r["first_lock"]))
    return "\n".join(lines)


def table_lateral_sweep(spec: VehicleSpec, offsets=(0.0, 0.1, 0.2, 0.3)) -> str:
    lines = ["| 载重横向偏移 | 左右制动力差 | 横摆角 | 横向跑偏 |", "|---|---|---|---|"]
    for dy in offsets:
        s = VehicleSpec(**{**spec.__dict__, "payload_y": dy})
        r = simulate_lateral(s)
        lines.append("| %.2f m | %.0f N | %.2f° | %.2f m |"
                     % (dy, r["brake_force_diff"], r["yaw_deg"], r["drift_m"]))
    return "\n".join(lines)
