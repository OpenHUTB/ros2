"""Checks for the Ubuntu documentation environment scripts."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_ubuntu_setup_creates_isolated_environment_and_installs_docs_requirements():
    content = (ROOT / "setup_ubuntu.sh").read_text(encoding="utf-8")

    assert "set -euo pipefail" in content
    assert 'python3 -m venv "${VENV_DIR}"' in content
    assert "requirements-docs.txt" in content
    assert "DEBIAN_FRONTEND=noninteractive" in content
    assert "import ensurepip" in content


def test_serve_script_requires_setup_and_enables_livereload():
    content = (ROOT / "serve.sh").read_text(encoding="utf-8")

    assert 'VENV_DIR="${DOCS_VENV:-${ROOT_DIR}/.venv}"' in content
    assert "mkdocs serve --livereload" in content
    assert "setup_ubuntu.sh" in content


def test_local_virtual_environment_is_ignored():
    content = (ROOT / ".gitignore").read_text(encoding="utf-8")
    assert ".venv/" in content