"""按回合生成训练、验证与测试数据。 Generate episode-level training, validation and test data."""
import argparse
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from simulation import Scenario, Simulation

FEATURES = ["sensor_range_m", "closing_speed_m_s", "height_error_m"]
TARGET = "accel_m_s2"


def generate(output, seed=2026, episodes=30):
    """仅在回合边界划分，清单记录参数与文件校验值。 Split at episode boundaries and record parameters/hashes."""
    if episodes < 5:
        raise ValueError("至少5个回合 / At least five episodes are required")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    order = rng.permutation(episodes)
    train_end, val_end = int(0.6 * episodes), int(0.8 * episodes)
    splits = {"train": sorted(order[:train_end].tolist()),
              "validation": sorted(order[train_end:val_end].tolist()),
              "test": sorted(order[val_end:].tolist())}
    scenarios = []
    for index in range(episodes):
        wall = float(rng.uniform(-0.5, 0.5))
        scenarios.append(Scenario(name=f"episode_{index:03d}", wall_x=wall,
                                  start_x=wall-float(rng.uniform(4, 9)),
                                  speed=float(rng.uniform(0, 2.5)),
                                  height=float(rng.uniform(0.8, 1.8)), duration=12))
    manifest = {"seed": seed, "mujoco_version": mujoco.__version__, "numpy_version": np.__version__,
                "timestep_s": 0.01, "controller": "range", "features": FEATURES, "target": TARGET,
                "feature_units": ["m", "m/s", "m"], "target_unit": "m/s^2", "splits": {}}
    for split, indices in splits.items():
        reports = []
        count = 0
        path = output / f"{split}.csv"
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = None
            for index in indices:
                scenario = scenarios[index]
                rows, report = Simulation(scenario, controller="range").run()
                report["parameters"] = asdict(scenario)
                reports.append(report)
                for row in rows:
                    record = {"episode_id": scenario.name, **row}
                    if writer is None:
                        writer = csv.DictWriter(stream, fieldnames=record, lineterminator="\n")
                        writer.writeheader()
                    writer.writerow(record)
                    count += 1
        manifest["splits"][split] = {"file": path.name, "rows": count, "episodes": reports,
                                      "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("datasets"))
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--episodes", type=int, default=30)
    args = parser.parse_args()
    result = generate(args.output, args.seed, args.episodes)
    print(json.dumps({name: {"rows": value["rows"], "episodes": len(value["episodes"]), "sha256": value["sha256"]}
                      for name, value in result["splits"].items()}, indent=2))
