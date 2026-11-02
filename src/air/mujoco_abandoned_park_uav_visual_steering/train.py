"""Train and evaluate the lightweight CNN / 训练并评价轻量卷积网络。"""

from __future__ import annotations

import argparse
import csv
import json
import random
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from sklearn.metrics import accuracy_score, confusion_matrix, f1_score
from torch import nn
from torch.utils.data import DataLoader, Dataset

try:
    from .mujoco_abandoned_park_uav_visual_steering.model import LightweightSteeringCNN
except ImportError:
    from mujoco_abandoned_park_uav_visual_steering.model import LightweightSteeringCNN


class FrameDataset(Dataset[tuple[torch.Tensor, int]]):
    def __init__(self, rows: list[dict[str, str]], root: Path, labels: tuple[str, ...]) -> None:
        self.rows = rows
        self.root = root
        self.label_to_id = {label: index for index, label in enumerate(labels)}

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int) -> tuple[torch.Tensor, int]:
        row = self.rows[index]
        with Image.open(self.root / row["image_path"]) as image:
            image = image.convert("RGB").resize((160, 90))
            array = np.asarray(image, dtype=np.float32) / 255.0
        tensor = torch.from_numpy(array.transpose(2, 0, 1)).contiguous()
        return tensor, self.label_to_id[row["label"]]


def load_rows(dataset_dir: Path) -> list[dict[str, str]]:
    with (dataset_dir / "metadata.csv").open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def evaluate_model(model: LightweightSteeringCNN, loader: DataLoader, device: str) -> dict[str, object]:
    model.eval()
    actual: list[int] = []
    predicted: list[int] = []
    latencies: list[float] = []
    with torch.no_grad():
        for images, targets in loader:
            images = images.to(device)
            start = time.perf_counter()
            logits = model(images)
            elapsed = (time.perf_counter() - start) / len(images)
            latencies.append(elapsed * 1000.0)
            predicted.extend(logits.argmax(dim=1).cpu().tolist())
            actual.extend(targets.tolist())
    matrix = confusion_matrix(actual, predicted, labels=list(range(len(model.labels))))
    return {
        "accuracy": float(accuracy_score(actual, predicted)),
        "macro_f1": float(f1_score(actual, predicted, average="macro", zero_division=0)),
        "confusion_matrix": matrix.tolist(),
        "inference_latency_ms_mean": float(np.mean(latencies)),
        "inference_latency_ms_p95": float(np.percentile(latencies, 95)),
        "samples": len(actual),
    }


def train(dataset_dir: Path, output_dir: Path, epochs: int = 8, seed: int = 250102) -> dict[str, object]:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    device = "cpu"
    rows = load_rows(dataset_dir)
    labels = LightweightSteeringCNN.labels
    train_rows = [row for row in rows if row["split"] == "train"]
    validation_rows = [row for row in rows if row["split"] == "validation"]
    test_rows = [row for row in rows if row["split"] == "test"]
    if not train_rows or not validation_rows or not test_rows:
        raise ValueError("all three episode-isolated splits are required / 三个回合隔离集合均不能为空")
    model = LightweightSteeringCNN().to(device)
    train_loader = DataLoader(FrameDataset(train_rows, dataset_dir, labels), batch_size=16, shuffle=True, generator=torch.Generator().manual_seed(seed))
    validation_loader = DataLoader(FrameDataset(validation_rows, dataset_dir, labels), batch_size=32, shuffle=False)
    test_loader = DataLoader(FrameDataset(test_rows, dataset_dir, labels), batch_size=32, shuffle=False)
    class_counts = np.bincount([labels.index(row["label"]) for row in train_rows], minlength=len(labels))
    weights = torch.tensor(len(train_rows) / np.maximum(class_counts, 1), dtype=torch.float32)
    criterion = nn.CrossEntropyLoss(weight=weights)
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    history: list[dict[str, float]] = []
    best_score = -1.0
    output_dir.mkdir(parents=True, exist_ok=True)
    for epoch in range(1, epochs + 1):
        model.train()
        losses: list[float] = []
        for images, targets in train_loader:
            optimizer.zero_grad(set_to_none=True)
            loss = criterion(model(images), targets)
            loss.backward()
            optimizer.step()
            losses.append(float(loss.item()))
        validation = evaluate_model(model, validation_loader, device)
        record = {"epoch": float(epoch), "loss": float(np.mean(losses)), "validation_accuracy": float(validation["accuracy"]), "validation_macro_f1": float(validation["macro_f1"])}
        history.append(record)
        if float(validation["macro_f1"]) >= best_score:
            best_score = float(validation["macro_f1"])
            torch.save({"state_dict": model.state_dict(), "image_height": 90, "image_width": 160, "labels": labels, "seed": seed}, output_dir / "lightweight_steering_cnn.pt")
    model = LightweightSteeringCNN.load_checkpoint(output_dir / "lightweight_steering_cnn.pt", device)
    metrics = evaluate_model(model, test_loader, device)
    result = {"seed": seed, "epochs": epochs, "labels": labels, "train_samples": len(train_rows), "validation_samples": len(validation_rows), "test_samples": len(test_rows), "history": history, "test": metrics}
    (output_dir / "metrics.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Train CNN / 训练 CNN")
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("models/weights"))
    parser.add_argument("--epochs", type=int, default=8)
    args = parser.parse_args()
    print(json.dumps(train(args.dataset.resolve(), args.output.resolve(), args.epochs), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


