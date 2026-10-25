"""独立测试集评估。 / Held-out test-set evaluation."""

import argparse
import json
from pathlib import Path

import numpy as np

from .model import RiskSpeedMLP, save_metadata


def evaluate(dataset: Path, model_dir: Path):
    with np.load(dataset / "test.npz") as data:
        x, target, risk = data["x"], data["target_speed"], data["risk"]
    model = RiskSpeedMLP.load(model_dir / "risk_speed_mlp.npz")
    probability, predicted_speed = model.predict(x)
    predicted_risk = probability.argmax(1)
    confusion = np.zeros((3, 3), dtype=int)
    for expected, actual in zip(risk, predicted_risk):
        confusion[int(expected), int(actual)] += 1
    result = {"test_samples": int(len(x)),
              "risk_accuracy": float(np.mean(predicted_risk == risk)),
              "speed_mae_mps": float(np.mean(np.abs(predicted_speed - target))),
              "confusion_matrix": confusion.tolist(),
              "model": "risk_speed_mlp"}
    save_metadata(model_dir / "test_metrics.json", result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description="评估仓储无人机模型 / Evaluate warehouse UAV model")
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--model", type=Path, default=Path("model"))
    args = parser.parse_args(argv)
    print(json.dumps(evaluate(args.dataset, args.model), indent=2))
