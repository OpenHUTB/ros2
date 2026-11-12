#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键实验入口：启动 Gazebo 无界面仿真 → 等待导航结束 → 自动生成真实数据图表。

先编译并 source：
    colcon build --packages-select diff_drive_astar_odom_nav
    source install/setup.bash
再运行：
    python3 main.py                 # 默认输出到 results/latest
    python3 main.py --out results/run_01 --timeout 150
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


def run_experiment(out_dir, timeout, gui):
    root = Path(__file__).resolve().parent
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    for name in ("status.json", "run_info.json", "metrics.json"):
        path = out_dir / name
        if path.exists():
            path.unlink()

    cmd = [
        "ros2", "launch", "diff_drive_astar_odom_nav", "main.launch.py",
        "gui:=%s" % ("true" if gui else "false"),
        "output_dir:=%s" % out_dir,
    ]
    log_path = out_dir / "launch.log"
    log_file = open(log_path, "w", encoding="utf-8")
    print("启动：", " ".join(cmd))
    proc = subprocess.Popen(
        cmd, stdout=log_file, stderr=subprocess.STDOUT,
        start_new_session=True, cwd=str(root), env=os.environ.copy())

    status_path = out_dir / "status.json"
    started = time.time()
    status = None
    try:
        while time.time() - started < timeout:
            if status_path.exists():
                time.sleep(0.5)  # 留出时间让 logger 结束写入
                status = json.loads(status_path.read_text(encoding="utf-8"))
                break
            if proc.poll() is not None:
                break
            time.sleep(0.5)
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        log_file.close()

    if status is None:
        print("未在 %.0f s 内收到导航状态，详见 %s" % (timeout, log_path))
        return None, out_dir

    print("导航状态：", status)
    metrics = subprocess.run(
        [sys.executable, str(root / "tools" / "analyze.py"), str(out_dir)],
        check=False, cwd=str(root), env=os.environ.copy())
    if metrics.returncode != 0:
        print("数据图生成失败，退出码 %d" % metrics.returncode)
        return None, out_dir
    return json.loads((out_dir / "metrics.json").read_text(encoding="utf-8")), out_dir


def main():
    parser = argparse.ArgumentParser(description="差速底盘 A* + 里程计闭环一键实验")
    parser.add_argument("--out", default="results/latest", help="结果目录")
    parser.add_argument("--timeout", type=float, default=120.0, help="等待导航结束的最长秒数")
    parser.add_argument("--gui", action="store_true", help="启动 Gazebo 图形界面")
    args = parser.parse_args()

    metrics, out_dir = run_experiment(args.out, args.timeout, args.gui)
    if metrics is None:
        sys.exit(1)
    print("结果目录：", out_dir)
    print(json.dumps(metrics, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
