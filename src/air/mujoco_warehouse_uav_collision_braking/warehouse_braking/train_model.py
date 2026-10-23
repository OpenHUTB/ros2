"""训练风险与速度模型。 / Train the risk and speed model."""

import argparse
import json
from pathlib import Path

import numpy as np

from .model import RiskSpeedMLP, save_metadata


def train(dataset: Path, output: Path, epochs=80, seed=42):
    with np.load(dataset / "train.npz") as data:
        x, target, risk = data["x"], data["target_speed"], data["risk"]
    model = RiskSpeedMLP(seed=seed)
    history = model.fit(x, target, risk, epochs=epochs)
    output.mkdir(parents=True, exist_ok=True)
    model.save(output / "risk_speed_mlp.npz")
    metadata = {"model": "RiskSpeedMLP", "input_size": 20, "hidden": [24, 16],
                "epochs": epochs, "seed": seed, "train_samples": len(x),
                "final_cross_entropy": history[-1], "feature_source": "warehouse_braking.dataset"}
    save_metadata(output / "training.json", metadata)
    return metadata


def main(argv=None):
    parser = argparse.ArgumentParser(description="训练仓储无人机MLP / Train warehouse UAV MLP")
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("model"))
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args(argv)
    if args.epochs <= 0:
        parser.error("训练轮数必须为正数 / Epochs must be positive")
    print(json.dumps(train(args.dataset, args.output, args.epochs, args.seed), indent=2))
