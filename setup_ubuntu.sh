#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${DOCS_VENV:-${ROOT_DIR}/.venv}"

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is required; install Python 3.8 or newer first." >&2
  exit 1
fi

if ! python3 -c "import ensurepip" >/dev/null 2>&1; then
  if [[ "${EUID}" -eq 0 ]]; then
    APT_PREFIX=()
  elif command -v sudo >/dev/null 2>&1; then
    APT_PREFIX=(sudo)
  else
    echo "python3-venv is missing and sudo is unavailable." >&2
    exit 1
  fi
  export DEBIAN_FRONTEND=noninteractive
  "${APT_PREFIX[@]}" apt-get update
  "${APT_PREFIX[@]}" apt-get install -y python3-venv
fi

python3 -m venv "${VENV_DIR}"
"${VENV_DIR}/bin/python" -m pip install --upgrade pip
"${VENV_DIR}/bin/python" -m pip install -r "${ROOT_DIR}/requirements-docs.txt"

printf '\nDocumentation environment is ready.\n'
printf 'Start the live site with:\n  %s\n' "${ROOT_DIR}/serve.sh"