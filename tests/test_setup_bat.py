"""Regression checks for the Windows setup script."""

from pathlib import Path


SETUP_SCRIPT = Path(__file__).resolve().parents[1] / "setup.bat"


def test_setup_script_uses_public_repository_without_embedded_credentials():
    content = SETUP_SCRIPT.read_text(encoding="utf-8")

    assert "https://github.com/OpenHUTB/ros2.git" in content
    assert "@git.code.tencent.com" not in content
    assert "OpenHUTB:" not in content


def test_setup_script_does_not_destroy_existing_worktree():
    content = SETUP_SCRIPT.read_text(encoding="utf-8")

    assert "git reset --hard" not in content
    assert "git clean -fdx" not in content
    assert "git pull --ff-only" in content
