#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地形跟随实验的命令行入口。

    python main.py                 # MuJoCo 闭环，跑四个算例 × 三种策略
    python main.py --no-sim        # 只跑一阶滞后运动学模型（不装 mujoco 也能跑）
    python main.py --case ridge_8ms --strategy dem_preview
    python main.py --sweep         # 速度扫描
    python main.py --json --out-dir results
"""
from __future__ import annotations

import argparse
import json
import math
import os
import platform
import sys
import time
from typing import Dict, List

from .core.cases import CASES, DEM_LEVELS, get_case, sweep_cases
from .core.control import STRATEGIES, steady_state_lag_m
from .core.evaluate import format_table
from .core.flyer import Flight, fly
from .core.terrain import list_terrains

DEFAULT_STRATEGIES = ("constant_msl", "reactive_agl", "dem_preview")


# --------------------------------------------------------------------------- #
# 工具
# --------------------------------------------------------------------------- #
def _json_safe(obj):
    """把 nan/inf 换成 None，保证产物是合法 JSON。"""
    if isinstance(obj, dict):
        return {k: _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_safe(v) for v in obj]
    if isinstance(obj, float):
        return None if (math.isnan(obj) or math.isinf(obj)) else obj
    if isinstance(obj, (int, str, bool)) or obj is None:
        return obj
    return str(obj)


def _row(case_name: str, flight: Flight) -> Dict:
    m = flight.metrics()
    return {
        "case": case_name,
        "strategy": flight.strategy,
        "min_clearance_m": m["min_clearance_m"],
        "rms_agl_error_m": m["rms_agl_error_m"],
        "max_agl_error_m": m["max_agl_error_m"],
        "time_below_safety_s": m["time_below_safety_s"],
        "max_climb_rate_mps": m["max_climb_rate_mps"],
        "crashed": m["crashed"],
    }


def _print_rows(rows: List[Dict], title: str) -> None:
    print(f"\n{title}")
    print(format_table(rows,
                       columns=("case", "strategy", "min_clearance_m", "rms_agl_error_m",
                                "max_agl_error_m", "time_below_safety_s",
                                "max_climb_rate_mps", "crashed"),
                       headers=("算例", "策略", "最小间隙m", "AGL_RMS_m",
                                "AGL_最大_m", "低于安全s", "最大爬升m/s", "撞地")))


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #
def run_cases(plant: str, strategies=DEFAULT_STRATEGIES, cases=CASES):
    rows, detail = [], []
    for case in cases:
        flights = []
        for name in strategies:
            t0 = time.time()
            f = fly(case, name, plant=plant)
            flights.append((f, time.time() - t0))
            rows.append(_row(case.name, f))
            print(f"  {case.name:<14s} {name:<14s} "
                  f"min_gap={rows[-1]['min_clearance_m']:7.2f} m  "
                  f"rms={rows[-1]['rms_agl_error_m']:5.2f} m  "
                  f"({flights[-1][1]:.1f}s)")
        detail.append({"case": case.as_dict(),
                       "flights": [{"strategy": f.strategy,
                                    "plant": f.plant,
                                    "metrics": _json_safe(f.metrics()),
                                    "wall_s": round(wall, 2)}
                                   for f, wall in flights]})
    return rows, detail


def run_dem_levels(plant: str, base_case: str = "ridge_8ms") -> List[Dict]:
    template = get_case(base_case)
    out = []
    for key, cfg, label in DEM_LEVELS:
        case = type(template)(
            name=f"{base_case}_{key}", title=f"{template.title} · {label}",
            terrain_name=template.terrain_name, speed_mps=template.speed_mps,
            agl_set_m=template.agl_set_m, safety_m=template.safety_m,
            length_m=template.length_m, ramp_s=template.ramp_s, seed=template.seed,
            dem=cfg, limits=template.limits, altimeter=template.altimeter)
        f = fly(case, "dem_preview", plant=plant)
        m = f.metrics()
        out.append({"dem": key, "label": label,
                    "min_clearance_m": m["min_clearance_m"],
                    "rms_agl_error_m": m["rms_agl_error_m"],
                    "crashed": m["crashed"]})
        print(f"  DEM={key:<10s} min_gap={m['min_clearance_m']:7.2f} m  "
              f"rms={m['rms_agl_error_m']:5.2f} m")
    return out


def run_sweep(plant: str, strategies=DEFAULT_STRATEGIES) -> List[Dict]:
    out = []
    for case in sweep_cases():
        for name in strategies:
            f = fly(case, name, plant=plant)
            m = f.metrics()
            out.append({"speed_mps": case.speed_mps, "strategy": name,
                        "required_climb_rate_mps": case.required_climb_rate(),
                        "min_clearance_m": m["min_clearance_m"],
                        "rms_agl_error_m": m["rms_agl_error_m"],
                        "crashed": m["crashed"]})
            print(f"  v={case.speed_mps:4.1f} m/s {name:<14s} "
                  f"min_gap={m['min_clearance_m']:7.2f} m  "
                  f"rms={m['rms_agl_error_m']:5.2f} m")
    return out


def run_ablation(plant: str) -> Dict[str, List[Dict]]:
    """两组消融：比例环的稳态滞后（验证解析式）与偏置积分快慢的取舍。"""
    from .core.control import DemPreview, ReactiveAGL

    # 1) 恒坡上的比例环：ki=0 时才符合 e = v·tanθ·(1/Kp + τ)
    slope_case = get_case("slope6_10ms")
    theory = steady_state_lag_m(slope_case.speed_mps,
                               math.tan(math.radians(slope_case.terrain.slope_deg)),
                               1.2, 0.17)
    lag_rows = []
    for ki in (0.0, 0.05, 0.25):
        strat = ReactiveAGL(slope_case.limits, kp=1.2, ki=ki)
        m = fly(slope_case, strat, plant=plant).metrics()
        lag_rows.append({"ki": ki, "rms_agl_error_m": m["rms_agl_error_m"],
                         "min_clearance_m": m["min_clearance_m"]})
        print(f"  恒坡 6°/10 m/s  ki={ki:<5g} AGL RMS={m['rms_agl_error_m']:.3f} m")
    print(f"  解析（纯比例）v·tanθ·(1/Kp+τ) = {theory:.3f} m")
    lag_rows = [dict(r, theory_m=theory) for r in lag_rows]

    # 2) DEM 偏置积分：快了平地更准，但山脊间隙掉
    ridge_case = get_case("ridge_8ms")
    slope_rms = None
    trim_rows = []
    for ki in (0.005, 0.02, 0.05, 0.10):
        m_ridge = fly(ridge_case, DemPreview(ridge_case.limits, ki=ki),
                      plant=plant).metrics()
        m_slope = fly(slope_case, DemPreview(slope_case.limits, ki=ki),
                      plant=plant).metrics()
        slope_rms = m_slope["rms_agl_error_m"]
        trim_rows.append({"ki": ki,
                          "ridge_min_clearance_m": m_ridge["min_clearance_m"],
                          "slope_rms_agl_m": slope_rms})
        print(f"  ki={ki:<5g} 山脊最小间隙={m_ridge['min_clearance_m']:6.2f} m  "
              f"恒坡 AGL RMS={slope_rms:.2f} m")
    return {"lag": lag_rows, "trim": trim_rows}


def main(argv=None):
    """命令入口：把「缺 MuJoCo 后端」这类可预期的 RuntimeError 变成一句人话。

    直接用 `python main.py` 而仓库里没有 `src/air/uav_mujoco_sim` 时，
    以前会先抛一整屏 traceback 才看到提示；现在只打印提示并返回 1。
    """
    try:
        return _main(argv)
    except RuntimeError as exc:
        print(f"运行失败：{exc}", file=sys.stderr)
        return 1


def _main(argv=None):
    parser = argparse.ArgumentParser(
        description="低空地形跟随：恒定离地高度保持策略对比（MuJoCo 闭环）")
    parser.add_argument("--case", action="append", choices=[c.name for c in CASES],
                        help="只跑指定算例（可重复）")
    parser.add_argument("--strategy", action="append", choices=sorted(STRATEGIES),
                        help="只跑指定策略（可重复）")
    parser.add_argument("--plant", choices=("mujoco", "kinematic"), default="mujoco",
                        help="被控对象：真物理 / 一阶滞后运动学")
    parser.add_argument("--no-sim", action="store_true",
                        help="等价于 --plant kinematic（不装 mujoco 也能跑）")
    parser.add_argument("--sweep", action="store_true", help="跑速度扫描")
    parser.add_argument("--dem-levels", action="store_true", help="跑 DEM 质量三档")
    parser.add_argument("--ablation", action="store_true",
                        help="跑消融：比例环滞后解析式 + 偏置积分快慢")
    parser.add_argument("--list", action="store_true", help="列出算例/策略/地形")
    parser.add_argument("--json", action="store_true", help="同时写 JSON 结果")
    parser.add_argument("--out-dir", default="results", help="JSON 输出目录")
    args = parser.parse_args(argv)

    if args.list:
        print("算例：")
        for c in CASES:
            print(f"  {c.name:<14s} {c.title}")
        print("策略：")
        for name in sorted(STRATEGIES):
            print(f"  {name}")
        print("地形：")
        print("  " + ", ".join(list_terrains()))
        return 0

    plant = "kinematic" if args.no_sim else args.plant
    strategies = tuple(args.strategy) if args.strategy else DEFAULT_STRATEGIES
    cases = tuple(get_case(n) for n in args.case) if args.case else CASES

    print(f"地形跟随实验：plant={plant}，"
          f"{len(cases)} 个算例 × {len(strategies)} 种策略")
    t0 = time.time()
    rows, detail = run_cases(plant, strategies, cases)
    _print_rows(rows, "主表（真值口径）")

    dem_rows = run_dem_levels(plant) if args.dem_levels else []
    if dem_rows:
        print("\nDEM 质量三档（dem_preview，ridge_8ms）")
        print(format_table(dem_rows,
                           columns=("dem", "min_clearance_m", "rms_agl_error_m", "crashed"),
                           headers=("DEM", "最小间隙m", "AGL_RMS_m", "撞地")))

    sweep_rows = run_sweep(plant, strategies) if args.sweep else []
    if sweep_rows:
        print("\n速度扫描（山脊地形，爬升率上限 "
              f"{CASES[0].limits.climb_rate_mps:g} m/s）")
        print(format_table(sweep_rows,
                           columns=("speed_mps", "strategy", "required_climb_rate_mps",
                                    "min_clearance_m", "rms_agl_error_m", "crashed"),
                           headers=("v_m/s", "策略", "需求爬升m/s", "最小间隙m",
                                    "AGL_RMS_m", "撞地")))

    # 解析对照：恒坡上的稳态滞后
    print("\n解析对照（恒坡 6°，理论稳态滞后 = v·tanθ·(1/Kp + τ)）")
    theory = []
    for v in (4.0, 8.0, 10.0):
        theory.append({"speed_mps": v,
                       "theory_m": steady_state_lag_m(v, math.tan(math.radians(6.0)),
                                                      1.2, 0.17)})
        print(f"  v={v:4.1f} m/s  →  {theory[-1]['theory_m']:.2f} m")

    ablation = {}
    if args.ablation:
        print("\n消融（比例环滞后 / DEM 偏置积分）")
        ablation = run_ablation(plant)

    payload = _json_safe({
        "plant": plant,
        "python": platform.python_version(),
        "elapsed_s": round(time.time() - t0, 2),
        "cases": detail,
        "rows": rows,
        "dem_levels": dem_rows,
        "sweep": sweep_rows,
        "theory_lag": theory,
        "ablation": ablation,
    })
    if args.json:
        os.makedirs(args.out_dir, exist_ok=True)
        path = os.path.join(args.out_dir, "terrain_follow.json")
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=2)
        print(f"\n结果已写入 {path}")
    print(f"总耗时 {payload['elapsed_s']:.1f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
