#!/usr/bin/env bash
# 运行多 AUV 协同围捕示例（默认 dry-run，不需要 HoloOcean）
set -e
python3 -m multi_agent_hunting.main --seed 42 --max-steps 300 "$@"
