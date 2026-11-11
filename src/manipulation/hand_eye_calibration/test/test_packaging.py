# -*- coding: utf-8 -*-
"""Packaging smoke tests."""
from pathlib import Path


def test_package_import():
    import hand_eye_calibration
    assert hand_eye_calibration.__version__


def test_model_file_exists():
    root = Path(__file__).resolve().parents[1]
    assert (root / "models" / "franka_emika_panda" / "scene_hand_eye.xml").is_file()