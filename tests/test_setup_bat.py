"""Regression checks for the Windows setup script."""

import re
from pathlib import Path


SETUP_SCRIPT = Path(__file__).resolve().parents[1] / "setup.bat"


def test_setup_script_uses_public_repository_without_embedded_credentials():
    content = SETUP_SCRIPT.read_text(encoding="utf-8")

    assert "https://github.com/OpenHUTB/ros2.git" in content
    assert "@git.code.tencent.com" not in content
    assert "OpenHUTB:" not in content
    assert not re.search(r"\bsshpass(?:\.exe)?\s+-p\b", content, re.IGNORECASE)
    assert not re.search(
        r"\b[a-z][a-z0-9+.-]*://[^/\s:@]+:[^/\s@]+@",
        content,
        re.IGNORECASE,
    )
    assert not re.search(
        r"\b(?:"
        r"10(?:\.\d{1,3}){3}|"
        r"192\.168(?:\.\d{1,3}){2}|"
        r"172\.(?:1[6-9]|2\d|3[01])(?:\.\d{1,3}){2}"
        r")\b",
        content,
    )


def test_remote_roscore_requires_environment_configuration():
    content = SETUP_SCRIPT.read_text(encoding="utf-8")

    assert "if not defined OPENHUTB_REMOTE_USER" in content
    assert "if not defined OPENHUTB_REMOTE_HOST" in content
    assert "%OPENHUTB_REMOTE_USER%@%OPENHUTB_REMOTE_HOST%" in content


def test_setup_script_does_not_destroy_existing_worktree():
    content = SETUP_SCRIPT.read_text(encoding="utf-8")

    assert "git reset --hard" not in content
    assert "git clean -fdx" not in content
    assert "git pull --ff-only" in content
