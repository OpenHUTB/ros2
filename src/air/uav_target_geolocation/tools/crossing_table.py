#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""交会占优的临界开角：把正文"开角小于某个角时交会反而更差"这个数钉死。

做法：对每个单站零位偏置 σ_b，先算单站算例的 CEP50（与被扫的交会曲线同一
统计量），再把两站开角从 4° 扫到 60°，取交会 CEP50 第一次降到单站基线以下
的位置（相邻两点线性插值）。

    python tools/crossing_table.py
    python tools/crossing_table.py --mc 600 --json

结论（默认参数，n=1500）：临界角钉在 45° 上下，与 σ_b 几乎无关——交会的收益
（两条独立偏置平均，1/√2）和几何放大（小开角把角误差摊成大位移）是两个独立
因子，且都正比于偏置，交点自然不随偏置大小移动。

**口径陷阱**：单站基线必须按偏置重算。若把基线钉死在 0.46 m（那是 σ_b=0.20°
的单站值），会得到"偏置越小、临界角越小"的假象（0.05° → 9°、0.10° → 19°），
因为那实际是"不同偏置的曲线穿过同一条水平线"。本工具每个偏置都重算基线。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
_ROOT = _HERE.parent
for _p in (str(_ROOT), str(_ROOT.parent / "uav_mujoco_sim")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from uav_target_geolocation.core.analysis import (  # noqa: E402
    fusion_crossing_angle_deg, monte_carlo)
from uav_target_geolocation.core.cases import get_case, sweep_case  # noqa: E402

#: 扫描的开角网格（度）。步长 4° 够用：临界点靠插值，误差 ≲ 2°
ANGLES = np.arange(4.0, 61.0, 4.0)
#: 扫描的单站零位偏置（度，1σ）。0.20° 是本模块算例的取值
BIASES = (0.05, 0.10, 0.20)
SIGMA_DEG = 0.30
SAMPLES = 120


def sweep_one(bias_deg: float, mc: int, seed: int) -> dict:
    """一个偏置档：单站基线 + 交会曲线 + 临界角（同一统计量 CEP50）。"""
    base = replace(get_case("single_station"), bias_deg=float(bias_deg))
    single = monte_carlo(base, n=mc, seed=seed)["single"]

    fused = []
    for a in ANGLES:
        res = monte_carlo(sweep_case(float(a), sigma_deg=SIGMA_DEG,
                                     bias_deg=float(bias_deg)),
                          n=mc, seed=seed, samples_per_station=SAMPLES)
        fused.append(float(res["fused"]["cep50_m"]) if res["fused"] else float("nan"))

    fused = np.asarray(fused)
    cross = fusion_crossing_angle_deg(ANGLES, fused, single["cep50_m"])
    # 90° 正交不在网格里，单独补一个点（正文表格要用）
    orth = monte_carlo(sweep_case(90.0, sigma_deg=SIGMA_DEG,
                                  bias_deg=float(bias_deg)),
                       n=mc, seed=seed, samples_per_station=SAMPLES)["fused"]
    return {
        "bias_deg": round(float(bias_deg), 3),
        "single_cep50_m": round(float(single["cep50_m"]), 4),
        "crossing_deg": cross,
        "fused_cep50_at_12deg_m": round(float(np.interp(12.0, ANGLES, fused)), 4),
        "fused_cep50_at_90deg_m": round(float(orth["cep50_m"]), 4),
        "curve": {"angle_deg": [round(float(a), 1) for a in ANGLES],
                  "fused_cep50_m": [round(float(v), 4) for v in fused]},
    }


def build(mc: int, seed: int) -> dict:
    return {"mc": int(mc), "seed": int(seed), "sigma_deg": SIGMA_DEG,
            "samples_per_station": SAMPLES,
            "rows": [sweep_one(b, mc, seed) for b in BIASES]}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="单站偏置 vs 交会临界开角")
    ap.add_argument("--mc", type=int, default=1500,
                    help="每档蒙特卡洛次数（400 时 CEP50 噪声约 5%%，"
                         "临界角会在 42~53° 之间乱跳）")
    ap.add_argument("--seed", type=int, default=1, help="随机流种子")
    ap.add_argument("--json", action="store_true", help="把结果 JSON 打到标准输出")
    args = ap.parse_args(argv)

    data = build(args.mc, args.seed)
    print(f"{'单站偏置σ/°':>12}{'单站 CEP50/m':>14}{'临界开角/°':>12}"
          f"{'12° 交会/m':>12}{'90° 交会/m':>12}")
    for r in data["rows"]:
        cross = "—" if r["crossing_deg"] is None else f"{r['crossing_deg']:.1f}"
        print(f"{r['bias_deg']:>12.2f}{r['single_cep50_m']:>14.3f}"
              f"{cross:>12}{r['fused_cep50_at_12deg_m']:>12.3f}"
              f"{r['fused_cep50_at_90deg_m']:>12.3f}")

    out_dir = _ROOT / "results"
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "crossing_table.json"
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8")
    print(f"[ok] → {path}")
    if args.json:
        print(json.dumps(data, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
