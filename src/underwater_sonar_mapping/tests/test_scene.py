"""Regression tests for deterministic sonar scene rendering."""

import sys
from pathlib import Path

import numpy as np

MODULE_ROOT = Path(__file__).resolve().parents[1]
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

from underwater_sonar_mapping.scene import create_scene, render_scene  # noqa: E402


def test_scene_is_deterministic_and_has_expected_geometry():
    scene = create_scene(grid_points=25, extent=20.0)

    assert scene.surface_x.shape == (25, 25)
    assert scene.surface_y.shape == (25, 25)
    assert scene.depth.shape == (25, 25)
    assert np.isfinite(scene.depth).all()
    assert scene.depth.min() >= 7.0
    assert scene.depth.max() <= 13.0
    assert len(scene.obstacles) == 3


def test_scene_rejects_invalid_grid_size():
    with np.testing.assert_raises(ValueError):
        create_scene(grid_points=1)


def test_headless_render_writes_png(tmp_path):
    output = render_scene(tmp_path / "sonar.png", dpi=72)

    assert output.exists()
    assert output.stat().st_size > 1_000
    assert output.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"