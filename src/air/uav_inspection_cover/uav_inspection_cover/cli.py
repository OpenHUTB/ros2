#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CLI orchestration shared by ``python main.py`` and ``ros2 run``.

流水线：观察点 → 最近邻航线（基线） → 2-opt 改进航线 → 覆盖校验 →
MuJoCo 闭环真飞（前 4 段）→ JSON 落盘 + 控制台报告。
"""
from __future__ import annotations

import argparse
import json
import os
import time
from typing import List

from .core.checker import cover_report
from .core.flyer import fly_legs
from .core.geometry import Params
from .core.planner import Route, build_cases, make_route, nearest_neighbor, \
    route_cost, two_opt

CASES = ("case_grid_short", "case_grid_long", "case_grid_river")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="输电走廊无人机巡检覆盖规划（MuJoCo 闭环验证）")
    p.add_argument("--case", default="all",
                   help="case 名（%s）或 all" % ", ".join(CASES))
    p.add_argument("--no-sim", action="store_true", help="跳过 MuJoCo 闭环真飞")
    p.add_argument("--json", default="", help="结果 JSON 输出路径")
    p.add_argument("--progress", action="store_true", help="打印仿真进度")
    return p


def _route_payload(route: Route, params: Params) -> dict:
    rep = cover_report(route.viewpoints, route.waypoints, params.cover_radius_m)
    return dict(
        waypoints=[[round(x, 1), round(y, 1), round(z, 1)]
                   for x, y, z in route.waypoints],
        distance_m=round(route.distance_m, 1),
        duration_s=round(route.duration_s(params), 1),
        coverage_rate=round(rep.rate, 3),
        coverage_missed=rep.missed,
        tower_order=list(route.order))


def run_case(name: str, params: Params, sim: bool = True,
             progress: bool = False) -> dict:
    """跑一个 case 的完整流程，返回可序列化的 mission 字典。"""
    corridor = build_cases()[name]
    print("[planner] %s: %d towers, start (%.0f, %.0f)"
          % (name, len(corridor.towers), *corridor.start))

    nn = nearest_neighbor(corridor)
    baseline = make_route(corridor, params, nn)
    print("[planner] %s baseline (nearest-neighbor): %.1f m, %.1f s, order %s"
          % (name, baseline.distance_m, baseline.duration_s(params), nn))

    improved_order = two_opt(nn, route_cost(corridor, params))
    improved = make_route(corridor, params, improved_order)
    print("[planner] improved (2-opt): %.1f m, %.1f s, order %s"
          % (improved.distance_m, improved.duration_s(params), improved_order))
    print("[planner] 2-opt gain: %.1f m (%.1f%%)"
          % (baseline.distance_m - improved.distance_m,
             100.0 * (baseline.distance_m - improved.distance_m)
             / baseline.distance_m))

    for tag, route in (("baseline", baseline), ("improved", improved)):
        rep = cover_report(route.viewpoints, route.waypoints,
                           params.cover_radius_m)
        print("[checker] %s coverage: %.1f%%, missed %s"
              % (tag, 100.0 * rep.rate, rep.missed or "none"))

    mission = dict(
        case=name,
        title=corridor.title,
        towers=len(corridor.towers),
        zones=[list(z) for z in corridor.zones],
        route_baseline=_route_payload(baseline, params),
        route_improved=_route_payload(improved, params),
        fly=None)

    if sim:
        out = {}
        for tag, route in (("baseline", baseline), ("improved", improved)):
            print("[flyer] %s: flying first %d legs ..." % (tag, params.legs))
            st = fly_legs(route.waypoints, params, legs=params.legs,
                          hover_at=route.hover_at, progress=progress)
            print("[flyer] %s: planned %.1f m, actual %.1f m, "
                  "arrival error %.2f m, wall %.1f s"
                  % (tag, st.planned_m, st.actual_m, st.arrival_error_m,
                     st.wall_time_s))
            out[tag] = st.as_dict()
        mission["fly"] = out
    return mission


def run_all(names: List[str], params: Params, sim: bool = True,
            progress: bool = False) -> List[dict]:
    return [run_case(n, params, sim=sim, progress=progress) for n in names]


def to_json(missions: List[dict], path: str) -> str:
    payload = dict(outcome="ok", tool="uav_inspection_cover",
                   missions=missions)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return path


def format_summary(missions: List[dict]) -> str:
    """控制台汇总表（README 结果块直接从这里粘贴）。"""
    lines = ["=" * 76,
             "%-18s %8s %9s %9s %8s %9s %9s" %
             ("case", "towers", "base_m", "impr_m", "gain%", "base_s",
              "impr_s")]
    for m in missions:
        b, i = m["route_baseline"], m["route_improved"]
        gain = 100.0 * (b["distance_m"] - i["distance_m"]) / b["distance_m"]
        lines.append("%-18s %8d %9.1f %9.1f %8.1f %9.1f %9.1f" %
                     (m["case"], m["towers"], b["distance_m"],
                      i["distance_m"], gain, b["duration_s"],
                      i["duration_s"]))
        fly = m.get("fly")
        if fly:
            lines.append("    fly baseline: actual %.1f m / err %.2f m | "
                         "improved: actual %.1f m / err %.2f m"
                         % (fly["baseline"]["actual_m"],
                            fly["baseline"]["arrival_error_m"],
                            fly["improved"]["actual_m"],
                            fly["improved"]["arrival_error_m"]))
    lines.append("=" * 76)
    return "\n".join(lines)


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    params = Params()
    t0 = time.time()

    if args.case == "all":
        missions = run_all(list(CASES), params, sim=not args.no_sim,
                           progress=args.progress)
    elif args.case in CASES:
        missions = [run_case(args.case, params, sim=not args.no_sim,
                             progress=args.progress)]
    else:
        print("unknown case %r, choose from %s or all"
              % (args.case, ", ".join(CASES)))
        return 2

    print()
    print(format_summary(missions))

    path = args.json or os.path.join("results", "inspection_result.json")
    print("[planner] results written to %s (%.1f s total)"
          % (to_json(missions, path), time.time() - t0))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
