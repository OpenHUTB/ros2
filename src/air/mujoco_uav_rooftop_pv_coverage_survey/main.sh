#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${PYTHON:-python3}"
MODE="${1:-all}"
OUTPUT="${2:-${ROOT}/artifacts/day2}"
exec "${PYTHON}" "${ROOT}/main.py" --mode "${MODE}" --output "${OUTPUT}"
