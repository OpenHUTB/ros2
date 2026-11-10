#!/bin/bash
# uav_l1_altitude_stabilization 一键入口
set -e

echo "=== UAV L1 Altitude Stabilization Demo ==="
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 检查 Python
if ! command -v python3 &> /dev/null; then
    echo "Error: python3 not found"
    exit 1
fi

# 运行演示
bash run_demo.sh
