"""多场景参考与学习控制对照。 / Compare reference and learned control across scenes."""

import argparse
import json
from pathlib import Path

import numpy as np

from .model import RiskSpeedMLP
from .simulation import DT, WarehouseSimulation, expert_target


def run_learned(model, scenario, speed=1.2, duration=8.0):
    sim = WarehouseSimulation(scenario, speed=speed)
    previous = None
    predicted_risks = []
    reference_risks = []
    for _ in range(round(duration / DT)):
        ranges = sim.ranges()
        closing = np.zeros(9) if previous is None else np.clip((previous - ranges) / DT, -10, 10)
        features = np.concatenate([ranges, closing, [sim.data.qvel[0], speed]])[None, :]
        probability, learned_target = model.predict(features)
        reference_target, reference_risk = expert_target(ranges, sim.data.qvel[0], speed)
        predicted_risks.append(int(probability[0].argmax()))
        reference_risks.append(reference_risk)
        sim.step(min(float(learned_target[0]), reference_target))
        previous = ranges
    return {"collision": bool(sim.collision), "final_speed_mps": float(sim.data.qvel[0]),
            "risk_agreement": float(np.mean(np.asarray(predicted_risks) == reference_risks)),
            "steps": len(predicted_risks)}


def benchmark(model_path: Path, output: Path):
    model = RiskSpeedMLP.load(model_path)
    result = {scenario: run_learned(model, scenario) for scenario in ("static", "crossing", "narrow")}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description="多场景模型对照 / Multi-scenario model benchmark")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("benchmark.json"))
    args = parser.parse_args(argv)
    print(json.dumps(benchmark(args.model, args.output), indent=2))
