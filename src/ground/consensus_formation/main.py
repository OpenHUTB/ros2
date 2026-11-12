#!/usr/bin/env python3
"""多机器人 Consensus 编队 — 启动入口。"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from consensus_formation import run_demo

if __name__ == "__main__":
    run_demo()
