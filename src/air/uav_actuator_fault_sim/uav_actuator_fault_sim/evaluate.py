# -*- coding: utf-8 -*-
"""批量评估：同一场景下各种故障的对比表、CSV 落盘、全矩阵扫描。"""

from __future__ import annotations

import csv
from typing import List, Optional, Sequence

from .metrics import FlightResult, aggregate, group_summary
from .runner import RunConfig, run_many
from .scenario import FAULT_ONLY, PHYSICAL_SCENES, FlightCase, make_case

#: 对比表的列顺序（与 CSV 表头一致）
COLUMNS = [
    ("scenario", "故障"),
    ("crashed", "坠机"),
    ("min_altitude", "最低高度m"),
    ("crash_delay", "坠地延时s"),
    ("touchdown_speed", "触地速度m/s"),
    ("horiz_drift", "水平漂移m"),
    ("pos_rmse", "位置RMSE"),
    ("control_effort", "控制能量"),
    ("final_yaw_rate", "末段偏航dps"),
]


def comparison_cases(scene: str = "open_ground", seed: int = 0, onset: float = 2.0,
                     rotor: int = 0) -> List[FlightCase]:
    """同一物理场景、同一注入时刻与桨序号下，healthy + 6 种故障的对照组。"""
    cases = [make_case(scene, "healthy", seed=seed, rotor=rotor, onset=onset)]
    for fault in FAULT_ONLY:
        cases.append(make_case(scene, fault, seed=seed, rotor=rotor, onset=onset))
    return cases


def run_comparison(scene: str = "open_ground", seed: int = 0, onset: float = 2.0,
                   rotor: int = 0, cfg: Optional[RunConfig] = None) -> List[FlightResult]:
    """跑一组对照实验。"""
    return run_many(comparison_cases(scene, seed, onset, rotor), cfg=cfg)


def run_matrix(seeds: Sequence[int] = (0,), cfg: Optional[RunConfig] = None
               ) -> List[FlightResult]:
    """跑完整矩阵：5 物理场景 × 7 故障场景 × len(seeds)。"""
    cases: List[FlightCase] = []
    for seed in seeds:
        for scene in PHYSICAL_SCENES:
            for fault in ("healthy",) + FAULT_ONLY:
                cases.append(make_case(scene, fault, seed=seed))
    return run_many(cases, cfg=cfg)


def _cell(row: dict, key: str) -> str:
    val = row.get(key, "")
    if key == "crashed":
        return "是" if val else "否"
    return str(val)


def format_table(results: Sequence[FlightResult]) -> str:
    """把一组结果排成等宽中文表格。"""
    rows = [r.summary() for r in results]
    widths = []
    for key, title in COLUMNS:
        # 中文按 2 个字符宽度估算，保证列对齐
        body = max([len(_cell(row, key)) for row in rows] + [1])
        widths.append(max(body, len(title) * 2))
    header = "  ".join(title.ljust(w) for (_, title), w in zip(COLUMNS, widths))
    sep = "  ".join("-" * w for w in widths)
    lines = [header, sep]
    for row in rows:
        cells = []
        for (key, _), w in zip(COLUMNS, widths):
            text = _cell(row, key)
            pad = w - len(text)
            cells.append(text + " " * max(0, pad))
        lines.append("  ".join(cells))
    return "\n".join(lines)


def format_summary(results: Sequence[FlightResult]) -> str:
    """按故障场景分组的生存率汇总。"""
    groups = group_summary(results, key="scenario")
    lines = [f"{'故障':<22}{'样本':>5}{'存活率':>9}{'平均最低高度m':>15}{'平均坠地延时s':>15}"]
    lines.append("-" * 68)
    for name, agg in groups.items():
        lines.append(
            f"{name:<22}{int(agg['n']):>5}{agg['survival_rate']:>9.2f}"
            f"{agg['mean_min_alt']:>15.2f}{agg['mean_crash_delay']:>15.2f}"
        )
    total = aggregate(results)
    lines.append("-" * 68)
    lines.append(f"{'合计':<22}{int(total['n']):>5}{total['survival_rate']:>9.2f}"
                 f"{total['mean_min_alt']:>15.2f}{total['mean_crash_delay']:>15.2f}")
    return "\n".join(lines)


def write_csv(results: Sequence[FlightResult], path: str) -> str:
    """把逐条指标写成 CSV，返回写出的路径。"""
    import os

    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    rows = [r.summary() for r in results]
    fieldnames = ["tag"] + [k for k, _ in COLUMNS] + [
        "safe_landing", "onset", "duration"]
    with open(path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for r, row in zip(results, rows):
            out = dict(row)
            out["tag"] = f"{r.scene}__{r.scenario}"
            writer.writerow({k: out.get(k, "") for k in fieldnames})
    return path
