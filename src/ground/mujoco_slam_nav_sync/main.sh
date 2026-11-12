#!/usr/bin/env bash
# 一键跑通：训练 -> 三模式对照 -> 漂移扫描 -> 出图出 GIF
set -e
cd "$(dirname "$0")"
python3 main.py --demo "$@"
