#!/usr/bin/env bash
# 一行包装：进入本目录并运行 python3 main.py "$@"
cd "$(dirname "$0")" && python3 main.py "$@"
