#!/bin/bash
# CARLA 停车场场景生成启动脚本

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 默认连接地址，可通过参数覆盖
HOST="${1:-192.168.43.1}"

echo "连接 CARLA 服务器: $HOST:2000"
python3 "$SCRIPT_DIR/main.py" --host "$HOST"
