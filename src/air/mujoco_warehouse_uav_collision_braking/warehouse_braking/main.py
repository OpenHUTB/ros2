"""统一运行入口。 / Unified execution entry point."""

import argparse
import json
from pathlib import Path

from .simulation import DT, WarehouseSimulation, expert_target


def run(args):
    sim = WarehouseSimulation(args.scenario, speed=args.speed)
    trace = []
    for _ in range(round(args.duration / DT)):
        ranges = sim.ranges()
        target, risk = expert_target(ranges, sim.data.qvel[0], args.speed)
        trace.append({"time_s": round(float(sim.data.time), 3),
                      "x_m": float(sim.position[0]), "speed_mps": float(sim.data.qvel[0]),
                      "ranges_m": ranges.tolist(), "target_speed_mps": target, "risk": risk})
        sim.step(target)
    args.output.mkdir(parents=True, exist_ok=True)
    result = {"scenario": args.scenario, "steps": len(trace), "collision": bool(sim.collision),
              "final_x_m": float(sim.position[0]), "final_speed_mps": float(sim.data.qvel[0]),
              "controller": "stopping_distance_reference"}
    (args.output / "trace.json").write_text(json.dumps(trace, indent=2), encoding="utf-8")
    (args.output / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    if args.render:
        from .visualize import render_run
        render_run(args.output, args.scenario, args.duration, args.speed)
    print("运行结果 / Run result: " + json.dumps(result))


def main(argv=None):
    parser = argparse.ArgumentParser(description="仓储巡检测距与制动 / Warehouse ranging and braking")
    parser.add_argument("--scenario", choices=["static", "crossing", "narrow"], default="static")
    parser.add_argument("--speed", type=float, default=1.2)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--output", type=Path, default=Path("results"))
    parser.add_argument("--render", action="store_true", help="保存仿真动画 / Save simulation animation")
    parser.add_argument("--collect", action="store_true", help="采集监督数据 / Collect supervised data")
    parser.add_argument("--episodes", type=int, default=30)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args(argv)
    if not (0 < args.speed <= 3 and 0 < args.duration <= 120):
        parser.error("速度须在 (0,3]，时长须在 (0,120]。 / Invalid speed or duration.")
    if args.collect:
        from .dataset import collect
        manifest = collect(args.output, args.episodes, args.seed, args.duration)
        print(json.dumps(manifest["splits"], indent=2))
    else:
        run(args)
