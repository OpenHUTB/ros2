"""Checks for the Ubuntu documentation environment scripts."""

import os
import shutil
import subprocess
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _cygpath(bash):
    if os.name != "nt":
        return None
    cygpath = shutil.which("cygpath")
    if cygpath is None:
        candidate = Path(bash).resolve().parent.parent / "usr" / "bin" / "cygpath.exe"
        if candidate.is_file():
            cygpath = str(candidate)
    if cygpath is None:
        raise unittest.SkipTest("cygpath is required for Git Bash path conversion")
    return cygpath


def _bash_path(path, bash):
    cygpath = _cygpath(bash)
    if cygpath is None:
        return str(path)
    return subprocess.check_output([cygpath, "-u", str(path)], text=True).strip()


def _bash_path_list(paths, bash):
    cygpath = _cygpath(bash)
    if cygpath is None:
        return paths
    return subprocess.check_output([cygpath, "-p", paths], text=True).strip()


def _run_setup_with_fake_python(tmp_path, python_version):
    bash = shutil.which("bash")
    if bash is None:
        raise unittest.SkipTest("bash is required for the setup script test")

    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    fake_python = bin_dir / "python3"
    fake_python.write_text(
        """#!/usr/bin/env bash
if [[ "$1" == "-c" && "$2" == *"sys.version_info"* ]]; then
  IFS=. read -r major minor patch <<< "${FAKE_PYTHON_VERSION:-3.14}"
  if (( major < 3 || (major == 3 && minor < 8) )); then
    exit 1
  fi
  exit 0
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
    bash_bin_dir = _bash_path(bin_dir, bash)
    bash_path = _bash_path_list(env.get("PATH", ""), bash)
    env["PATH"] = f"{bash_bin_dir}:{bash_path}"
    env["DOCS_VENV"] = _bash_path(tmp_path / "venv", bash)
    env["FAKE_PYTHON_VERSION"] = python_version
    return subprocess.run(
        [bash, _bash_path(ROOT / "setup_ubuntu.sh", bash)],
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


def test_ubuntu_setup_rejects_python_older_than_minimum(tmp_path):
    result = _run_setup_with_fake_python(tmp_path, "3.7.17")

    assert result.returncode == 1
    assert "Python 3.8 or newer is required." in result.stderr


def test_ubuntu_setup_accepts_python_minimum(tmp_path):
    result = _run_setup_with_fake_python(tmp_path, "3.8.0")

    assert result.returncode == 0, result.stderr
    assert "Documentation environment is ready." in result.stdout


def test_ubuntu_setup_accepts_python_newer_than_minimum(tmp_path):
    result = _run_setup_with_fake_python(tmp_path, "3.9.0")

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
