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

from .models import TinyImageController, TinyMLP, TinyUNet, TinyWaypointPlanner
from .planning import planning_features, route_waypoint_target


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


def train_end_to_end(dataset: Path, epochs: int = 12, seed: int = 20261016) -> dict[str, Any]:
    """Train RGB-to-command model and report held-out command MSE / 训练图像到控制网络并报告测试误差。"""
    _validate_training_data(dataset, epochs)
    torch.manual_seed(seed)
    model = TinyImageController()
    optimizer = torch.optim.Adam(model.parameters(), lr=2e-3)
    train_rows = _records(dataset, "train")
    images = torch.stack([_image_tensor(dataset / row["rgb_path"]) for row in train_rows])
    targets = torch.tensor(
        [[row["expert_command"][key] for key in ("forward_speed", "yaw_rate")] for row in train_rows],
        dtype=torch.float32,
    )
    for _ in range(epochs):
        optimizer.zero_grad()
        loss = F.mse_loss(model(images), targets)
        loss.backward()
        optimizer.step()
    test_rows = _records(dataset, "test")
    test_images = torch.stack([_image_tensor(dataset / row["rgb_path"]) for row in test_rows])
    test_targets = torch.tensor(
        [[row["expert_command"][key] for key in ("forward_speed", "yaw_rate")] for row in test_rows],
        dtype=torch.float32,
    )
    with torch.no_grad():
        predictions = model(test_images)
    return {
        "mse": float(F.mse_loss(predictions, test_targets)),
        "epochs": epochs,
        "train_frames": len(train_rows),
        "test_frames": len(test_rows),
        "input": "rgb_image",
        "output": ["forward_speed", "yaw_rate"],
        "checkpoint": _save_checkpoint(model, dataset, "end_to_end.pt", epochs, seed),
    }


def train_planning(dataset: Path, epochs: int = 100, seed: int = 20261016) -> dict[str, Any]:
    """Imitate route lookahead targets, with complete sequences held out from training.

    学习航线专家的局部航点偏移；测试序列不参与梯度更新或输入归一化。
    """
    _validate_training_data(dataset, epochs)
    train_rows, test_rows = _records(dataset, "train"), _records(dataset, "test")
    for row in train_rows + test_rows:
        if not row.get("sequence_id"):
            raise ValueError("planning training requires sequence_id for sequence isolation")
    train_sequences = sorted({row["sequence_id"] for row in train_rows})
    test_sequences = sorted({row["sequence_id"] for row in test_rows})
    if set(train_sequences) & set(test_sequences):
        raise ValueError("train and test sequences overlap / 训练与测试序列重叠")

    def tensors(rows):
        state = [[row["expert_input"][key] for key in
                  ("cross_track_error", "heading_error", "distance_to_turn")] for row in rows]
        return (torch.tensor([planning_features(*values) for values in state], dtype=torch.float32),
                torch.tensor([route_waypoint_target(*values) for values in state], dtype=torch.float32))

    torch.manual_seed(seed)
    model = TinyWaypointPlanner()
    optimizer = torch.optim.Adam(model.parameters(), lr=3e-3)
    features, targets = tensors(train_rows)
    for _ in range(epochs):
        optimizer.zero_grad()
        loss = F.mse_loss(model(features), targets)
        loss.backward()
        optimizer.step()
    model.eval()
    test_features, test_targets = tensors(test_rows)
    with torch.inference_mode():
        predictions = model(test_features)
        waypoint_error = torch.linalg.vector_norm(predictions - test_targets, dim=1)
    contract = {"input": ["cross_track_error_m", "heading_error_rad", "lookahead_distance_m"],
                "output": ["body_forward_offset_m", "body_left_offset_m"],
                "target": "route_expert_2m_lookahead_waypoint",
                "train_sequences": train_sequences, "test_sequences": test_sequences}
    return {**contract, "epochs": epochs, "train_frames": len(train_rows), "test_frames": len(test_rows),
            "mse": float(F.mse_loss(predictions, test_targets)),
            "waypoint_error_m": float(waypoint_error.mean()),
            "checkpoint": _save_checkpoint(model, dataset, "planning.pt", epochs, seed, contract)}
