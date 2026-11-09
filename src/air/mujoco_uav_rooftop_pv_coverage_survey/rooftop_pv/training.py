"""Small, reproducible training and evaluation routines."""

from __future__ import annotations

import json
from pathlib import Path
import time
from typing import Any

import numpy as np
from PIL import Image
import torch
from torch.nn import functional as F

from .models import TinyMLP, TinyUNet


def _save_checkpoint(model, dataset: Path, filename: str, epochs: int, seed: int, metadata=None) -> str:
    """Save trained parameters with their model contract / 保存训练权重及模型契约。"""
    dataset.mkdir(parents=True, exist_ok=True)
    torch.save({
        "format_version": 1,
        "model": type(model).__name__,
        "epochs": epochs,
        "seed": seed,
        "state_dict": model.state_dict(),
        **(metadata or {}),
    }, dataset / filename)
    return filename


def _validate_training_data(dataset: Path, epochs: int) -> None:
    if epochs < 1:
        raise ValueError("epochs must be positive / 训练轮数必须为正")
    for split in ("train", "test"):
        if not _records(dataset, split):
            raise ValueError(f"dataset has no {split} frames / 数据集缺少 {split} 帧")


def _records(dataset: Path, split: str) -> list[dict[str, Any]]:
    rows = [json.loads(line) for line in (dataset / "manifest.jsonl").read_text(encoding="utf-8").splitlines()]
    return [row for row in rows if row["split"] == split]


def _image_tensor(path: Path) -> torch.Tensor:
    array = np.asarray(Image.open(path).convert("RGB"), dtype=np.float32) / 255.0
    return torch.from_numpy(array.transpose(2, 0, 1))


def segmentation_loss(logits: torch.Tensor, masks: torch.Tensor) -> torch.Tensor:
    """BCE plus soft Dice loss."""
    bce = F.binary_cross_entropy_with_logits(logits, masks)
    probabilities = torch.sigmoid(logits)
    intersection = (probabilities * masks).sum()
    dice = (2.0 * intersection + 1.0) / (probabilities.sum() + masks.sum() + 1.0)
    return bce + (1.0 - dice)


def segmentation_metrics(prediction: torch.Tensor, target: torch.Tensor) -> dict[str, float]:
    pred = prediction.bool()
    truth = target.bool()
    intersection = float((pred & truth).sum())
    union = float((pred | truth).sum())
    tp = intersection
    fp = float((pred & ~truth).sum())
    fn = float((~pred & truth).sum())
    return {
        "iou": (intersection / union) if union else 1.0,
        "dice": (2.0 * intersection / (2.0 * intersection + fp + fn)) if (tp + fp + fn) else 1.0,
        "pixel_accuracy": float((pred == truth).float().mean()),
        "area_relative_error": abs(float(pred.float().mean() - truth.float().mean())) / max(float(truth.float().mean()), 1e-6),
    }


def train_segmentation(dataset: Path, epochs: int = 2, seed: int = 20261016) -> dict[str, Any]:
    _validate_training_data(dataset, epochs)
    torch.manual_seed(seed)
    model = TinyUNet()
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    train_rows = _records(dataset, "train")
    for _ in range(epochs):
        for row in train_rows:
            image = _image_tensor(dataset / row["rgb_path"])[None]
            mask = (_image_tensor(dataset / row["mask_path"])[None, :1] > 0).float()
            optimizer.zero_grad()
            loss = segmentation_loss(model(image), mask)
            loss.backward()
            optimizer.step()
    test_rows = _records(dataset, "test")
    metrics: list[dict[str, float]] = []
    elapsed = []
    with torch.no_grad():
        for row in test_rows:
            image = _image_tensor(dataset / row["rgb_path"])[None]
            mask = (_image_tensor(dataset / row["mask_path"])[None, :1] > 0)
            start = time.perf_counter()
            prediction = torch.sigmoid(model(image)) > 0.5
            elapsed.append((time.perf_counter() - start) * 1000.0)
            metrics.append(segmentation_metrics(prediction, mask))
    result = {key: float(np.mean([item[key] for item in metrics])) for key in metrics[0]}
    result["inference_ms"] = float(np.mean(elapsed))
    result.update({"epochs": epochs, "train_frames": len(train_rows), "test_frames": len(test_rows)})
    result["checkpoint"] = _save_checkpoint(model, dataset, "segmentation.pt", epochs, seed)
    return result


def train_control(dataset: Path, epochs: int = 30, seed: int = 20261016) -> dict[str, Any]:
    _validate_training_data(dataset, epochs)
    torch.manual_seed(seed)
    model = TinyMLP()
    optimizer = torch.optim.Adam(model.parameters(), lr=2e-3)
    train_rows = _records(dataset, "train")
    features = torch.tensor([[r["expert_input"][k] for k in ("cross_track_error", "heading_error", "distance_to_turn")] for r in train_rows], dtype=torch.float32)
    targets = torch.tensor([[r["expert_command"][k] for k in ("forward_speed", "yaw_rate")] for r in train_rows], dtype=torch.float32)
    for _ in range(epochs):
        optimizer.zero_grad()
        loss = F.mse_loss(model(features), targets)
        loss.backward()
        optimizer.step()
    test_rows = _records(dataset, "test")
    test_features = torch.tensor([[r["expert_input"][k] for k in ("cross_track_error", "heading_error", "distance_to_turn")] for r in test_rows], dtype=torch.float32)
    test_targets = torch.tensor([[r["expert_command"][k] for k in ("forward_speed", "yaw_rate")] for r in test_rows], dtype=torch.float32)
    with torch.no_grad():
        predictions = model(test_features)
    return {"mse": float(F.mse_loss(predictions, test_targets)), "epochs": epochs, "train_frames": len(train_rows), "test_frames": len(test_rows),
            "checkpoint": _save_checkpoint(model, dataset, "control.pt", epochs, seed)}




