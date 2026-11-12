"""Train and evaluate the elbow activation MLP from episode CSV files."""

import argparse
import csv
import json
from pathlib import Path
from typing import Iterable

import numpy as np

from .episode import split_episode_ids, write_episode_split
from .mlp import ActivationMLP, FEATURES, TARGETS, score


def read_rows(directory: Path):
    rows = []
    for path in sorted(Path(directory).glob("*.csv")):
        with path.open(encoding="utf-8", newline="") as stream:
            rows.extend(csv.DictReader(stream))
    if not rows:
        raise ValueError("no episode CSV files found")
    return rows


def load_or_create_split(
    data_dir: Path, episode_ids: Iterable[str], seed: int | None = None
):
    data_dir = Path(data_dir)
    episode_ids = set(episode_ids)
    manifest_path = data_dir / "episode_split.json"
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest_seed = manifest.get("seed")
        if (
            isinstance(manifest_seed, bool)
            or not isinstance(manifest_seed, int)
            or manifest_seed < 0
        ):
            raise ValueError("episode_split.json field 'seed' must be a non-negative integer")
        if seed is not None and seed != manifest_seed:
            raise ValueError(
                f"seed {seed} does not match episode_split.json seed {manifest_seed}"
            )
        partitions = manifest.get("partitions")
        names = ("train", "validation", "test")
        if not isinstance(partitions, dict) or any(
            not isinstance(partitions.get(name), list)
            or not partitions[name]
            or not all(isinstance(episode_id, str) for episode_id in partitions[name])
            for name in names
        ):
            raise ValueError("episode_split.json must contain non-empty partitions")
        assigned_ids = [
            episode_id for name in names for episode_id in partitions[name]
        ]
        if len(assigned_ids) != len(set(assigned_ids)):
            raise ValueError("episode_split.json partitions must be disjoint")
        if set(assigned_ids) != episode_ids:
            raise ValueError(
                "episode_split.json episode ids do not match the available CSV files"
            )
        return partitions, manifest_seed

    seed = 25012 if seed is None else seed
    partitions = split_episode_ids(sorted(episode_ids), seed)
    write_episode_split(data_dir, seed, partitions)
    return partitions, seed


def train_model(data_dir: Path, model_path: Path, seed: int | None = None):
    rows = read_rows(data_dir)
    episode_ids = sorted({row["episode_id"] for row in rows})
    partitions, seed = load_or_create_split(data_dir, episode_ids, seed)

    def arrays(names):
        selected = [row for row in rows if row["episode_id"] in names]
        features = np.asarray(
            [[float(row[key]) for key in FEATURES] for row in selected]
        )
        targets = np.asarray(
            [[float(row[key]) for key in TARGETS] for row in selected]
        )
        return features, targets

    train_x, train_y = arrays(partitions["train"])
    validation_x, validation_y = arrays(partitions["validation"])
    test_x, test_y = arrays(partitions["test"])

    model = ActivationMLP(seed=seed)
    loss = model.fit(
        train_x, train_y, validation=(validation_x, validation_y)
    )
    model.save(model_path)
    metrics = {
        "train": score(model, train_x, train_y).__dict__,
        "validation": score(model, validation_x, validation_y).__dict__,
        "test": score(model, test_x, test_y).__dict__,
        "partitions": partitions,
        "best_validation_loss": loss,
    }
    metrics_path = Path(model_path).with_suffix(".metrics.json")
    metrics_path.write_text(
        json.dumps(metrics, indent=2) + "\n", encoding="utf-8"
    )
    return metrics


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Train elbow activation MLP / 训练肘关节激活 MLP"
    )
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--seed", type=int)
    args = parser.parse_args(argv)
    args.model.parent.mkdir(parents=True, exist_ok=True)
    metrics = train_model(args.data, args.model, args.seed)
    print(json.dumps(metrics, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
