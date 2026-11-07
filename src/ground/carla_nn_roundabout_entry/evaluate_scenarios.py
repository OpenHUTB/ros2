#!/usr/bin/env python3
"""按多个随机种子汇总环岛仿真指标。"""

from __future__ import annotations

import argparse
import json
import statistics
from collections.abc import Iterable

from main import run_headless


def evaluate_scenarios(seeds: Iterable[int], steps: int = 360) -> dict[str, object]:
    seed_list = list(seeds)
    if not seed_list:
        raise ValueError("至少需要一个随机种子")
    if steps <= 0:
        raise ValueError("仿真步数必须为正数")

    results = [{"seed": seed, **run_headless(seed, steps)} for seed in seed_list]
    collisions = [int(result["collision_frames"]) for result in results]
    clearances = [float(result["minimum_clearance_m"]) for result in results]
    return {
        "steps_per_scenario": steps,
        "scenario_count": len(results),
        "scenarios_with_collisions": sum(value > 0 for value in collisions),
        "mean_collision_frames": round(statistics.mean(collisions), 3),
        "worst_collision_frames": max(collisions),
        "worst_minimum_clearance_m": round(min(clearances), 3),
        "results": results,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="汇总多个固定种子的环岛仿真指标")
    parser.add_argument("--seeds", nargs="+", type=int, default=[3, 7, 11, 19, 23])
    parser.add_argument("--steps", type=int, default=360)
    args = parser.parse_args()
    print(json.dumps(evaluate_scenarios(args.seeds, args.steps), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
