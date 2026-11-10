#!/bin/bash
# 运行 L1 高度控制阶跃响应仿真
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Starting UAV L1 altitude stabilization simulation..."
python3 scripts/l1_altitude_node.py
