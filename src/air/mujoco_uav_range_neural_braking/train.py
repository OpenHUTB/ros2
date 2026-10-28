"""训练并评估 MLP 制动控制器。 Train and evaluate the MLP braking controller."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from mlp import MLPRegressor


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    features = np.asarray([[float(row["sensor_range_m"]), float(row["closing_speed_m_s"])] for row in rows])
    target = np.asarray([float(row["accel_m_s2"]) for row in rows])
    return features, target


def evaluate(model, features, target):
    prediction = model.predict(features)
    error = prediction - target
    return {"mae_m_s2": float(np.mean(np.abs(error))), "rmse_m_s2": float(np.sqrt(np.mean(error ** 2))),
            "max_abs_error_m_s2": float(np.max(np.abs(error))), "samples": len(target)}


def train(data_dir, output):
    data_dir, output = Path(data_dir), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    train_x, train_y = read_csv(data_dir / "train.csv")
    val_x, val_y = read_csv(data_dir / "validation.csv")
    test_x, test_y = read_csv(data_dir / "test.csv")
    model = MLPRegressor(seed=2026).fit(train_x, train_y)
    model.save(output / "mlp_controller.json")
    metrics = {"train": evaluate(model, train_x, train_y), "validation": evaluate(model, val_x, val_y),
               "test": evaluate(model, test_x, test_y), "features": ["sensor_range_m", "closing_speed_m_s"],
               "target": "accel_m_s2", "epochs": 350, "hidden_size": 16}
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    return metrics


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=Path("datasets"))
    parser.add_argument("--output", type=Path, default=Path("models"))
    args = parser.parse_args()
    print(json.dumps(train(args.data, args.output), indent=2))
