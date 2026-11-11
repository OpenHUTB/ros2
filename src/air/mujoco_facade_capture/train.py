"""生成MuJoCo训练样本，训练控制网络并在独立立面评估。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from facade.dataset import TEST_SCENARIOS, TRAIN_SCENARIOS, collect_scenarios
from facade.learning import DenseController, control_features, train_controller
from facade.simulation import run_scan


def neural_policy(model: DenseController, distance: float):
    """构造仅由传感器、航点误差及速度组成的推理回调。"""
    def infer(error: np.ndarray, ranges: np.ndarray, velocity: np.ndarray) -> np.ndarray:
        features = control_features(error, ranges, velocity, distance)
        return model.predict(features)
    return infer


def evaluate(output: Path, epochs: int = 45) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    train_dir, test_dir = output / "train_data", output / "test_data"
    train_summary = collect_scenarios(train_dir, TRAIN_SCENARIOS)
    test_summary = collect_scenarios(test_dir, TEST_SCENARIOS)
    train = np.load(train_dir / "samples.npz")
    test = np.load(test_dir / "samples.npz")
    model = train_controller(train["features"], train["targets"], epochs=epochs)
    model_path = output / "facade_controller.json"
    model.save(model_path)
    command_error = model.predict(test["features"]) - test["targets"]
    neural_results, expert_results = [], []
    for scenario in TEST_SCENARIOS:
        name = scenario["style"]
        neural_results.append(run_scan(output / f"neural_{name}", policy=neural_policy(model, scenario["distance"]), **scenario))
        expert_results.append(run_scan(output / f"expert_{name}", **scenario))
    report = {
        "training_split": train_summary,
        "held_out_split": test_summary,
        "model": model_path.name,
        "held_out_expert_command_mae": float(np.mean(np.abs(command_error))),
        "held_out_expert_command_rmse": float(np.sqrt(np.mean(command_error ** 2))),
        "physical_scan_comparison": [
            {"style": s["style"], "controller": "neural", "completion_ratio": n["completion_ratio"],
             "distance_rmse_m": n["distance_rmse_m"], "maximum_speed_m_s": n["maximum_speed_m_s"]}
            for s, n in zip(TEST_SCENARIOS, neural_results)
        ] + [
            {"style": s["style"], "controller": "proportional", "completion_ratio": e["completion_ratio"],
             "distance_rmse_m": e["distance_rmse_m"], "maximum_speed_m_s": e["maximum_speed_m_s"]}
            for s, e in zip(TEST_SCENARIOS, expert_results)
        ],
    }
    (output / "evaluation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="训练并测试幕墙扫描速度控制神经网络")
    parser.add_argument("--output", type=Path, default=Path("output/training"))
    parser.add_argument("--epochs", type=int, default=45)
    args = parser.parse_args()
    print(json.dumps(evaluate(args.output, args.epochs), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
