#!/usr/bin/env python3
"""任务3 主入口：建图 -> A*规划 -> 自主导航 一键流程"""
import subprocess
import os
import sys

BASE = os.path.dirname(os.path.abspath(__file__))

def run(name):
    print(f"\n=== 运行 {name} ===")
    r = subprocess.run([sys.executable, os.path.join(BASE, name)], cwd=BASE)
    if r.returncode != 0:
        sys.exit(r.returncode)

if __name__ == "__main__":
    run('map_build.py')
    run('planner.py')
    run('navigate.py')
    print("\n=== 任务3 全流程完成！===")
