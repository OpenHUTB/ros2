"""扫描多组无人机场景并汇总结果。Sweep UAV scenarios and summarize outcomes."""

import argparse
import csv
import itertools
import json
import math
from pathlib import Path

from simulation import Scenario, Simulation


CSV_FIELDS = (
    "scenario", "controller", "start_x_m", "wall_x_m", "initial_speed_m_s", "height_m",
    "steps", "collision_steps", "min_clearance_m", "final_speed_m_s",
    "invalid_sensor_steps", "max_height_error_m",
)


def _values(value, name):
    values = tuple(float(item) for item in value.split(","))
    if not values or not all(math.isfinite(item) for item in values):
        raise ValueError(f"{name} 须为有限数值 / {name} must contain finite values")
    return values


def run_sweep(start_xs=(-8.0, -7.0), wall_xs=(0.0,), speeds=(0.0, 1.5), heights=(1.2,),
              controller="range", model_path=None):
    """遍历场景参数笛卡尔积并汇总仿真结果。Sweep the Cartesian product of scenario parameters."""
    combinations = tuple(itertools.product(start_xs, wall_xs, speeds, heights))
    if not combinations:
        raise ValueError("至少需要一个场景组合 / At least one scenario combination is required")
    results = []
    for index, (start_x, wall_x, speed, height) in enumerate(combinations):
        scenario = Scenario(
            name=f"scenario_{index:03d}", start_x=start_x, wall_x=wall_x, speed=speed, height=height,
        )
        _, report = Simulation(scenario, controller=controller, model_path=model_path).run()
        results.append({
            "scenario": scenario.name,
            "start_x_m": start_x,
            "wall_x_m": wall_x,
            "initial_speed_m_s": speed,
            "height_m": height,
            **report,
        })
    return {
        "controller": controller,
        "scenario_count": len(results),
        "collision_scenarios": sum(row["collision_steps"] > 0 for row in results),
        "max_collision_steps": max(row["collision_steps"] for row in results),
        "minimum_clearance_m": min(row["min_clearance_m"] for row in results),
        "scenarios": results,
    }


def write_sweep_csv(report, output):
    """将逐场景指标写入固定列序的 CSV。"""
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(report["scenarios"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=("reference", "range", "mlp"), default="range")
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--start-x", default="-8,-7", help="逗号分隔的起点 x 坐标 / comma-separated start x positions")
    parser.add_argument("--wall-x", default="0", help="逗号分隔的障碍物 x 坐标 / comma-separated wall x positions")
    parser.add_argument("--speed", default="0,1.5", help="逗号分隔的初速度 / comma-separated initial speeds")
    parser.add_argument("--height", default="1.2", help="逗号分隔的飞行高度 / comma-separated altitudes")
    parser.add_argument("--output", type=Path, default=None, help="可选 JSON 输出路径 / optional JSON output path")
    parser.add_argument("--csv-output", type=Path, default=None, help="可选逐场景 CSV 输出路径 / optional per-scenario CSV output path")
    args = parser.parse_args()
    result = run_sweep(
        _values(args.start_x, "start-x"), _values(args.wall_x, "wall-x"),
        _values(args.speed, "speed"), _values(args.height, "height"),
        controller=args.controller, model_path=args.model,
    )
    payload = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    if args.csv_output:
        write_sweep_csv(result, args.csv_output)
    print(payload)


if __name__ == "__main__":
    main()
