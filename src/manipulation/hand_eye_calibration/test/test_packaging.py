# -*- coding: utf-8 -*-
"""Packaging smoke tests."""
from pathlib import Path

import pytest


def test_package_import():
    import hand_eye_calibration
    assert hand_eye_calibration.__version__


def test_model_file_exists():
    root = Path(__file__).resolve().parents[1]
    assert (root / "models" / "franka_emika_panda" / "scene_hand_eye.xml").is_file()


def test_requirements_file_exists():
    root = Path(__file__).resolve().parents[1]
    assert (root / "requirements.txt").is_file()


def test_pose_targets_cover_yaw_roll_pitch():
    """回归测试：16 个位姿应覆盖 yaw/roll/pitch 三档角度（防止 pitch 索引恒为 0）。"""
    pytest.importorskip("mujoco")
    pytest.importorskip("cv2")
    import numpy as np

    from hand_eye_calibration import app

    poses = list(app.pose_targets(16))
    assert len(poses) == 16

    seen = {"yaw": set(), "roll": set(), "pitch": set()}
    for _, R in poses:
        best = None
        for yaw in (-20.0, 0.0, 20.0):
            for roll in (-8.0, 0.0, 8.0):
                for pitch in (-6.0, 0.0, 6.0):
                    Rc = (app.rotz(np.radians(yaw)) @ app.rotx(np.radians(roll))
                          @ app.roty(np.radians(pitch)) @ app.TOP_DOWN_R)
                    if np.allclose(Rc, R, atol=1e-9):
                        best = (yaw, roll, pitch)
        assert best is not None
        seen["yaw"].add(best[0])
        seen["roll"].add(best[1])
        seen["pitch"].add(best[2])
    assert seen["yaw"] == {-20.0, 0.0, 20.0}
    assert seen["roll"] == {-8.0, 0.0, 8.0}
    assert seen["pitch"] == {-6.0, 0.0, 6.0}
