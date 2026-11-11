"""Checks for the Ubuntu documentation environment scripts."""

import os
import shutil
import subprocess
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _run_setup_with_fake_python(tmp_path, too_old):
    bash = shutil.which("bash")
    if bash is None:
        raise unittest.SkipTest("bash is required for the setup script test")

    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    fake_python = bin_dir / "python3"
    fake_python.write_text(
        """#!/usr/bin/env bash
if [[ "$1" == "-c" && "$2" == *"sys.version_info"* ]]; then
  exit "${FAKE_PYTHON_TOO_OLD:-0}"
fi
if [[ "$1" == "-m" && "$2" == "venv" ]]; then
  mkdir -p "$3/bin"
  printf '#!/usr/bin/env bash\nexit 0\n' > "$3/bin/python"
  chmod +x "$3/bin/python"
  exit 0
fi
exit 0
""",
        encoding="utf-8",
    )
    fake_python.chmod(0o755)

    env = os.environ.copy()
    env["PATH"] = f"{bin_dir}{os.pathsep}{env.get('PATH', '')}"
    env["DOCS_VENV"] = str(tmp_path / "venv")
    env["FAKE_PYTHON_TOO_OLD"] = "1" if too_old else "0"
    return subprocess.run(
        [bash, str(ROOT / "setup_ubuntu.sh")],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )


def test_ubuntu_setup_creates_isolated_environment_and_installs_docs_requirements():
    content = (ROOT / "setup_ubuntu.sh").read_text(encoding="utf-8")

    assert "set -euo pipefail" in content
    assert "sys.version_info < (3, 8)" in content
    assert 'python3 -m venv "${VENV_DIR}"' in content
    assert "requirements-docs.txt" in content
    assert "DEBIAN_FRONTEND=noninteractive" in content
    assert "import ensurepip" in content


def test_ubuntu_setup_rejects_python_older_than_308(tmp_path):
    result = _run_setup_with_fake_python(tmp_path, too_old=True)

    assert result.returncode == 1
    assert "Python 3.8 or newer is required." in result.stderr


def test_ubuntu_setup_accepts_supported_python(tmp_path):
    result = _run_setup_with_fake_python(tmp_path, too_old=False)

    assert result.returncode == 0, result.stderr
    assert "Documentation environment is ready." in result.stdout


def test_serve_script_requires_setup_and_enables_livereload():
    content = (ROOT / "serve.sh").read_text(encoding="utf-8")

    assert 'VENV_DIR="${DOCS_VENV:-${ROOT_DIR}/.venv}"' in content
    assert "mkdocs serve --livereload" in content
    assert "setup_ubuntu.sh" in content


def test_local_virtual_environment_is_ignored():
    content = (ROOT / ".gitignore").read_text(encoding="utf-8")
    assert ".venv/" in content
