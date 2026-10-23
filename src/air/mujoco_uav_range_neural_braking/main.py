"""室内无人机制动入口。 Indoor UAV braking entry point."""
import argparse
import csv
import json
import time
from pathlib import Path

from simulation import Scenario, Simulation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="无窗口 / Run without a window")
    parser.add_argument("--output", type=Path, default=Path("results"), help="结果目录 / Results directory")
    parser.add_argument("--speed", type=float, default=1.5, help="初速度 / Initial speed, m/s")
    parser.add_argument("--controller", choices=("reference", "range", "mlp"), default="range", help="控制输入 / Control input")
    parser.add_argument("--model", type=Path, default=None, help="MLP 权重路径 / MLP model path")
    args = parser.parse_args()
    simulation = Simulation(Scenario(speed=args.speed), controller=args.controller, model_path=args.model)
    if args.headless:
        rows, summary = simulation.run()
    else:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(simulation.model, simulation.data) as viewer:
            def update(sim):
                if viewer.is_running():
                    viewer.sync()
                    time.sleep(sim.model.opt.timestep)
            rows, summary = simulation.run(update)
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if summary["collision_steps"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
