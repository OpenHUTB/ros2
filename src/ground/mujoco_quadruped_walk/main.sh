#!/bin/bash
cd "$(dirname "$0")"
echo "正在安装依赖..."
pip3 install -r requirements.txt
echo "启动四足机器人 trot 步态仿真..."
python3 scripts/main.py
