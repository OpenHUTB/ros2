"""Tests for package-resource resolution."""

from pathlib import Path

import pytest

from mujoco_diff_drive_slip_compensation.resources import resolve_package_file


def test_resolve_existing_file(tmp_path: Path):
    model_dir = tmp_path / 'models'
    model_dir.mkdir()
    model = model_dir / 'robot.xml'
    model.write_text('<mujoco/>', encoding='utf-8')

    assert resolve_package_file(tmp_path, 'models/robot.xml') == model.resolve()


def test_reject_path_escape(tmp_path: Path):
    outside = tmp_path.parent / 'outside.xml'
    outside.write_text('<mujoco/>', encoding='utf-8')

    with pytest.raises(ValueError):
        resolve_package_file(tmp_path, '../outside.xml')


def test_missing_file_has_clear_error(tmp_path: Path):
    with pytest.raises(FileNotFoundError, match='missing.xml'):
        resolve_package_file(tmp_path, 'models/missing.xml')
