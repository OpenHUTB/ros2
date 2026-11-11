# -*- coding: utf-8 -*-
"""Packaging smoke tests: no MuJoCo rendering / ROS runtime required."""
from pathlib import Path


def test_package_import():
    import drone_charging_nest
    assert drone_charging_nest.__version__


def test_model_file_exists():
    root = Path(__file__).resolve().parents[1]
    assert (root / "models" / "charging_nest.xml").is_file()