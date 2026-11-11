#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键实验：分别运行三次样条切换与直接切换，并生成对比分析。"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time

PKG_ROOT = os.path.dirname(os.path.abspath(__file__))
RESULTS_ROOT = os.path.join(PKG_ROOT, "results")


def wait_and_stop(proc, status_file, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if os.path.exists(status_file):
            break
        if proc.poll() is not None:
            break
        time.sleep(0.5)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        proc.kill()


def run_mode(mode, duration, output_dir):
    status_file = os.path.join(output_dir, "status.json")
    if os.path.exists(status_file):
        os.remove(status_file)
    os.makedirs(output_dir, exist_ok=True)
    cmd = [
        "ros2", "launch", "biped_cubic_spline_gait_switch", "main.launch.py",
        "mode:=%s" % mode,
        "output_dir:=%s" % output_dir,
        "duration:=%.1f" % duration,
    ]
    print("[main] 运行 %s：%s" % (mode, " ".join(cmd)))
    proc = subprocess.Popen(cmd, start_new_session=True)
    wait_and_stop(proc, status_file, timeout=duration + 50)
    if not os.path.exists(status_file):
        raise RuntimeError("%s 实验未生成 status.json，请检查终端输出" % mode)
    with open(status_file) as f:
        status = json.load(f)
    print("[main] %s 完成：%s" % (mode, status))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=34.0)
    parser.add_argument("--skip-direct", action="store_true",
                        help="只跑三次样条实验，不跑直接切换对照")
    args = parser.parse_args()

    spline_dir = os.path.join(RESULTS_ROOT, "run_spline")
    direct_dir = os.path.join(RESULTS_ROOT, "run_direct")

    run_mode("spline", args.duration, spline_dir)
    if not args.skip_direct:
        run_mode("direct", args.duration, direct_dir)

    analyze_cmd = [
        sys.executable, os.path.join(PKG_ROOT, "tools", "analyze.py"),
        "--data", spline_dir,
        "--baseline", direct_dir,
        "--config", os.path.join(PKG_ROOT, "config", "params.yaml"),
    ]
    print("[main] 生成分析：%s" % " ".join(analyze_cmd))
    subprocess.check_call(analyze_cmd)
    print("[main] 结果目录：%s" % spline_dir)


if __name__ == "__main__":
    main()
