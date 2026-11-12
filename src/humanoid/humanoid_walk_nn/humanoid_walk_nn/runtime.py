"""Project-local Python runtime bootstrap for the HNN GV project.

Ubuntu 20.04 / ROS Noetic ships Python 3.8, while MuJoCo 3.12 requires
Python >= 3.10.  The repository therefore keeps a private virtual environment
in ``<repo>/.venv`` (created by ``scripts/setup_env.sh``).  ROS nodes are
started by ``roslaunch`` with the system interpreter, detect that MuJoCo is
not importable, and re-exec themselves with the project interpreter.  No system
Python, shell profile or environment variable is modified.
"""

from __future__ import annotations

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
VENV_PYTHON = os.path.join(ROOT, ".venv", "bin", "python3")
MARKER = "HNN_VENV_ACTIVE"


def in_venv() -> bool:
    try:
        return os.path.realpath(sys.executable) == os.path.realpath(VENV_PYTHON)
    except OSError:
        return False


def ensure_venv(script: str | None = None) -> None:
    """Re-exec ``script`` with the project interpreter when necessary."""
    if os.environ.get(MARKER) == "1":
        return
    if sys.version_info >= (3, 10):
        try:
            import mujoco  # noqa: F401
            return
        except Exception:
            pass
    if not os.path.isfile(VENV_PYTHON):
        raise SystemExit(
            "MuJoCo 3.12 requires Python >= 3.10 but the project venv is missing.\n"
            "Run:  bash scripts/setup_env.sh\n"
            f"Expected interpreter: {VENV_PYTHON}"
        )
    if in_venv():
        return
    env = os.environ.copy()
    env[MARKER] = "1"
    target = os.path.realpath(script or sys.argv[0])
    os.execve(VENV_PYTHON, [VENV_PYTHON, target] + sys.argv[1:], env)


def add_repo_to_path() -> None:
    if ROOT not in sys.path:
        sys.path.insert(0, ROOT)
