#!/bin/bash
set -e
echo "=== UAV L1 Altitude Stabilization Demo ==="
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
bash run_demo.sh
