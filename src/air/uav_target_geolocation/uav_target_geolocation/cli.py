#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令行入口：跑算例、写结果 JSON、打印对照表。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from .core.analysis import monte_carlo, theory_single_cep50
from .core.cases import CASES, get_case
from .core.fusion import summarize


def _pkg_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _json_safe(obj):
    """把 nan / inf 换成 None：JSON 里它们不是合法数字，写进去别人读不了。"""
    import math

    if isinstance(obj, dict):
        return {k: _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_safe(v) for v in obj]
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, (np.floating,)):  # noqa: F821
        f = float(obj)
        return f if math.isfinite(f) else None
    return obj


def run_case(name: str, sim: bool = True, seed: int = 3, mc_n: int = 600,
             reps: int = 10) -> Dict[str, object]:
    """跑一个算例：解析打点 + （可选）MuJoCo 闭环。

    闭环跑 ``reps`` 个架次：每站的视轴零位偏置是"一整个架次不变"的，
    只飞一次的话拿到的只是偏置的一个抽样，浅交会这种几何本来就要看分布。
    """
    case = get_case(name)
    out: Dict[str, object] = {
        "case": case.name,
        "title": case.title,
        "truth_m": [round(float(x), 3) for x in case.target],
        "n_stations": case.n_stations,
        "sigma_deg": case.sigma_deg,
        "bias_deg": case.bias_deg,
        "wind_n": [round(float(case.wind_n[0]), 3),
                   round(float(case.wind_n[1]), 3)],
        "algorithm": monte_carlo(case, n=mc_n, seed=seed),
    }
    out["algorithm"]["theory_single_cep50_m"] = round(
        float(theory_single_cep50(case)), 4)

    if sim:
        from .core.flyer import fly_case, summarize_flight
        per_rep: List[Dict[str, object]] = []
        for i in range(int(reps)):
            res = fly_case(case, seed=seed + i)
            s = summarize_flight(case, res)
            per_rep.append({
                "seed": seed + i,
                "single_ref_m": s["single_ref_m"],
                "level_ref_m": s["level_ref_m"],
                "single_per_sample_mean_m": None if not s["per_sample"]["single"]
                else round(float(s["per_sample"]["single"]["mean_m"]), 4),
                "fused_m": s["station_fused"]["fused_m"],
                "fused_ok": s["station_fused"]["ok"],
                "max_hover_error_m": round(float(res.max_hover_error_m), 3),
                "sim_s": round(float(res.sim_s), 2),
                "wall_s": round(float(res.wall_s), 2),
                "per_station_m": s["per_station_m"],
                "stations": s["stations"],
            })

        def _col(key: str) -> List[float]:
            return [float(r[key]) for r in per_rep if r[key] is not None]

        fused_err = _col("fused_m")
        out["closed_loop"] = {
            "simulated": True,
            "reps": len(per_rep),
            "fused": summarize(fused_err) if fused_err else None,
            "single": summarize(_col("single_ref_m")) if _col("single_ref_m") else None,
            "level_assumption": summarize(_col("level_ref_m"))
            if _col("level_ref_m") else None,
            "per_sample": summarize(_col("single_per_sample_mean_m"))
            if _col("single_per_sample_mean_m") else None,
            "max_hover_error_m": round(
                max((r["max_hover_error_m"] for r in per_rep), default=float("nan")), 3),
            "sim_duration_s": round(sum(float(r["sim_s"]) for r in per_rep), 2),
            "wall_time_s": round(sum(float(r["wall_s"]) for r in per_rep), 2),
            # 逐架次明细（含每站偏置、悬停倾角），报告与文档里的数都从这里取
            "per_rep": per_rep,
        }
    else:
        out["closed_loop"] = {"simulated": False}
    return out


def _fmt(v, nd: int = 2) -> str:
    if v is None:
        return "—"
    try:
        return f"{float(v):.{nd}f}"
    except (TypeError, ValueError):
        return str(v)


def print_table(results: List[Dict[str, object]]) -> None:
    head = (f"{'算例':<24}{'站数':>4}{'开角°':>7}{'解析单站':>10}{'解析交会':>10}"
            f"{'假定水平':>10}{'实测单站':>10}{'实测交会':>10}")
    print(head)
    print("-" * 74)
    for r in results:
        alg = r["algorithm"]
        fused = alg.get("fused")
        closed = r.get("closed_loop") or {}
        sim = bool(closed.get("simulated"))
        cs = (closed.get("single") or {}) if sim else {}
        cf = (closed.get("fused") or {}) if sim else {}
        print(f"{r['case']:<24}{r['n_stations']:>4}"
              f"{_fmt(alg['intersection_deg'], 1):>7}"
              f"{_fmt(alg['single']['mean_m']):>10}"
              f"{_fmt(fused['mean_m'] if fused else None):>10}"
              f"{_fmt(alg['level_assumption']['mean_m']):>10}"
              f"{_fmt(cs.get('mean_m')):>10}"
              f"{_fmt(cf.get('mean_m')):>10}")
    print()
    reps = next((r["closed_loop"]["reps"] for r in results
                 if (r.get("closed_loop") or {}).get("simulated")), None)
    print("解析列：蒙特卡洛 600 次（几何 + 噪声）；"
          f"实测列：MuJoCo 闭环 {reps if reps else '—'} 个架次的位置误差均值。")
    print("「假定水平」= 拿指令姿态（水平）当真实姿态，悬停时的倾角全被丢掉。")


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="目标地理定位与多站交会：单站视线 vs 双站交会精度评估")
    ap.add_argument("--case", default="all",
                    help="算例名，或 all（默认）")
    ap.add_argument("--list", action="store_true", help="列出算例")
    ap.add_argument("--no-sim", action="store_true", help="只跑解析模式，不跑 MuJoCo")
    ap.add_argument("--json", action="store_true", help="把结果 JSON 打到标准输出")
    ap.add_argument("--out-dir", default=None, help="结果目录（默认 <包>/results）")
    ap.add_argument("--mc", type=int, default=600, help="蒙特卡洛次数")
    ap.add_argument("--reps", type=int, default=10, help="闭环架次数（每架次重新抽零位偏置）")
    args = ap.parse_args(argv)

    if args.list:
        for c in CASES:
            print(f"{c.name:<24} {c.title}  （{c.n_stations} 站，"
                  f"开角 {c.intersection_deg:.1f}°）")
        return 0

    names = [c.name for c in CASES] if args.case == "all" else [args.case]
    for n in names:
        get_case(n)  # 提前报错

    out_dir = Path(args.out_dir) if args.out_dir else _pkg_root() / "results"
    out_dir.mkdir(parents=True, exist_ok=True)

    results: List[Dict[str, object]] = []
    for n in names:
        r = run_case(n, sim=not args.no_sim, mc_n=args.mc, reps=args.reps)
        results.append(r)
        path = out_dir / f"{n}_result.json"
        path.write_text(
            json.dumps(_json_safe(r), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")
        print(f"[ok] {n} → {path}")

    print()
    print_table(results)
    if args.json:
        print(json.dumps(results, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
