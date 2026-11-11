"""Offline replay from another working directory / 从其他目录执行离线回放。"""

import json
import subprocess
import sys
from pathlib import Path

from PIL import Image

MODULE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_ROOT))


def test_main_replay_loads_checkpoint_without_simulation_config(tmp_path):
    import torch
    from mujoco_abandoned_park_uav_visual_steering.model import LightweightSteeringCNN

    model = LightweightSteeringCNN()
    checkpoint = tmp_path / "weights.pt"
    torch.save({"state_dict": model.state_dict(), "image_height": 90, "image_width": 160,
                "labels": model.labels}, checkpoint)
    images = tmp_path / "images"
    images.mkdir()
    Image.new("RGB", (160, 90), color=(90, 100, 110)).save(images / "frame.png")
    completed = subprocess.run(
        [sys.executable, str(MODULE_ROOT / "main.py"), "--mode", "replay",
         "--images", str(images), "--checkpoint", str(checkpoint),
         "--config", str(tmp_path / "absent.yaml")],
        cwd=tmp_path, capture_output=True, text=True, encoding="utf-8", timeout=60,
    )
    assert completed.returncode == 0, completed.stderr
    assert json.loads(completed.stdout)["frames"] == 1
