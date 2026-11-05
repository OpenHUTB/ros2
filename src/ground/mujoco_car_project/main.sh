#!/bin/bash
cd "$(dirname "$0")"
echo "正在安装依赖..."
pip3 install -r requirements.txt
echo "启动小车仿真..."
python3 scripts/main.py