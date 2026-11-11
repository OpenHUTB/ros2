"""Training interfaces for the visual steering CNN / 视觉转向 CNN 训练接口。"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

MODULE_ROOT = Path(__file__).resolve().parents[1] / "src" / "air" / "mujoco_abandoned_park_uav_visual_steering"
sys.path.insert(0, str(MODULE_ROOT))
sys.path.insert(0, str(MODULE_ROOT / "mujoco_abandoned_park_uav_visual_steering"))


def test_checkpoint_contains_four_class_contract(tmp_path: Path) -> None:
    torch = pytest.importorskip("torch")
    from model import LightweightSteeringCNN

    model = LightweightSteeringCNN()
    checkpoint = tmp_path / "checkpoint.pt"
    torch.save({"state_dict": model.state_dict(), "image_height": 90,
                "image_width": 160, "labels": model.labels}, checkpoint)
    loaded = LightweightSteeringCNN.load_checkpoint(checkpoint)
    assert tuple(loaded.labels) == ("LEFT", "STRAIGHT", "RIGHT", "STOP")
    assert loaded(torch.zeros((2, 3, 90, 160))).shape == (2, 4)
