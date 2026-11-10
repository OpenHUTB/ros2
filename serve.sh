#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${DOCS_VENV:-${ROOT_DIR}/.venv}"

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  echo "Documentation environment not found: ${VENV_DIR}" >&2
  echo "Run ${ROOT_DIR}/setup_ubuntu.sh first." >&2
  exit 1
fi

exec "${VENV_DIR}/bin/python" -m mkdocs serve --livereload "$@"