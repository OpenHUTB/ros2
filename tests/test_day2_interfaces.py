"""Day-two training, replay, and evaluation tests / 第二天训练、回放与评价测试。"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

MODULE_ROOT = Path(__file__).resolve().parents[1] / "src" / "air" / "mujoco_abandoned_park_uav_visual_steering"
sys.path.insert(0, str(MODULE_ROOT))
sys.path.insert(0, str(MODULE_ROOT / "mujoco_abandoned_park_uav_visual_steering"))


def test_ros_replay_and_evaluation_contract(tmp_path: Path) -> None:
    torch = pytest.importorskip("torch")
    from model import LightweightSteeringCNN
    from evaluation import evaluate
    from ros_node import VisualSteeringCore
    from PIL import Image

    checkpoint = tmp_path / "model.pt"
    model = LightweightSteeringCNN()
    torch.save({"state_dict": model.state_dict(), "image_height": 90, "image_width": 160}, checkpoint)
    core = VisualSteeringCore(checkpoint)
    command = core.predict(Image.new("RGB", (320, 180), color=(80, 100, 120)))
    assert command.label in model.labels
    assert 0.0 <= command.confidence <= 1.0
    result = evaluate(tmp_path / "scenarios.json")
    assert len(result["scenarios"]) == 3
    assert all(0.0 <= item["cnn_success_rate"] <= 1.0 for item in result["scenarios"])
    assert json.loads((tmp_path / "scenarios.json").read_text(encoding="utf-8"))["scenarios"]


def test_checkpoint_contains_four_class_contract(tmp_path: Path) -> None:
    torch = pytest.importorskip("torch")
    sys.path.insert(0, str(MODULE_ROOT / "mujoco_abandoned_park_uav_visual_steering"))
    from model import LightweightSteeringCNN

    model = LightweightSteeringCNN()
    checkpoint = tmp_path / "checkpoint.pt"
    torch.save({"state_dict": model.state_dict(), "image_height": 90, "image_width": 160, "labels": model.labels}, checkpoint)
    loaded = LightweightSteeringCNN.load_checkpoint(checkpoint)
    assert tuple(loaded.labels) == ("LEFT", "STRAIGHT", "RIGHT", "STOP")
    assert loaded(torch.zeros((2, 3, 90, 160))).shape == (2, 4)

