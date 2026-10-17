"""Train and evaluate the elbow activation MLP from episode CSV files."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from .episode import split_episode_ids
from .mlp import ActivationMLP, FEATURES, TARGETS, score


def read_rows(directory):
    rows = []
    for path in sorted(Path(directory).glob("*.csv")):
        with path.open(encoding="utf-8", newline="") as stream:
            rows.extend(list(csv.DictReader(stream)))
    if not rows: raise ValueError("no episode CSV files found")
    return rows


def train_model(data_dir: Path, model_path: Path, seed=25012):
    rows = read_rows(data_dir); ids = sorted({row["episode_id"] for row in rows}); partitions = split_episode_ids(ids, seed)
    def arrays(names):
        selected = [row for row in rows if row["episode_id"] in names]
        return np.asarray([[float(row[k]) for k in FEATURES] for row in selected]), np.asarray([[float(row[k]) for k in TARGETS] for row in selected])
    train_x, train_y = arrays(partitions["train"]); val_x, val_y = arrays(partitions["validation"]); test_x, test_y = arrays(partitions["test"])
    model = ActivationMLP(seed=seed); loss = model.fit(train_x, train_y, validation=(val_x, val_y)); model.save(model_path)
    metrics = {"train": score(model, train_x, train_y).__dict__, "validation": score(model, val_x, val_y).__dict__, "test": score(model, test_x, test_y).__dict__, "partitions": partitions, "best_validation_loss": loss}
    Path(model_path).with_suffix(".metrics.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    return metrics


def main(argv=None):
    parser = argparse.ArgumentParser(description="Train elbow activation MLP / 训练肘关节激活 MLP")
    parser.add_argument("--data", type=Path, required=True); parser.add_argument("--model", type=Path, required=True); parser.add_argument("--seed", type=int, default=25012)
    args = parser.parse_args(argv); args.model.parent.mkdir(parents=True, exist_ok=True); print(json.dumps(train_model(args.data, args.model, args.seed), indent=2)); return 0


if __name__ == "__main__": raise SystemExit(main())
