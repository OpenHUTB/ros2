"""Training metric and checkpoint-label regressions / 训练契约回归。"""
import sys
from pathlib import Path

import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mujoco_abandoned_park_uav_visual_steering.model import LightweightSteeringCNN
from train import evaluate_model


def test_macro_f1_includes_absent_classes():
    model = LightweightSteeringCNN()
    with torch.no_grad():
        for param in model.parameters():
            param.zero_()
        model.classifier[-1].bias[0] = 10.0
    batches = [(torch.zeros(2, 3, 90, 160), torch.zeros(2, dtype=torch.long))]
    metrics = evaluate_model(model, batches, "cpu")
    assert metrics["accuracy"] == 1.0
    assert metrics["macro_f1"] == 0.25
    assert len(metrics["confusion_matrix"]) == 4


@pytest.mark.parametrize("labels", [None, ("RIGHT", "STRAIGHT", "LEFT", "STOP")])
def test_checkpoint_rejects_missing_or_reordered_labels(tmp_path, labels):
    model = LightweightSteeringCNN()
    payload = {"state_dict": model.state_dict(), "image_height": 90, "image_width": 160}
    if labels is not None:
        payload["labels"] = labels
    checkpoint = tmp_path / "invalid.pt"
    torch.save(payload, checkpoint)
    with pytest.raises(ValueError, match="labels"):
        LightweightSteeringCNN.load_checkpoint(checkpoint)
