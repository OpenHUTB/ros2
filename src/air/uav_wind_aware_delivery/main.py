#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统一入口：信息、测试、训练、评估、ROS launch、录屏。"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
VENV_PY = os.environ.get("UAV_VENV_PYTHON", "/home/lxy/ground_nn_nav/.venv/bin/python")


def ensure_venv():
    """没有 mujoco 时尽量切到项目 venv。"""
    try:
        import mujoco  # noqa: F401
        return
    except Exception:
        if os.path.exists(VENV_PY) and os.path.realpath(sys.executable) != os.path.realpath(VENV_PY):
            os.execv(VENV_PY, [VENV_PY, os.path.abspath(__file__)] + sys.argv[1:])


def info():
    ensure_venv()
    import mujoco
    import numpy as np
    print(f"python   : {sys.version.split()[0]}")
    print(f"mujoco   : {mujoco.__version__}")
    print(f"numpy    : {np.__version__}")
    print(f"model    : {os.path.join(ROOT, 'models', 'uav_delivery.xml')}")
    print(f"predictor: {os.path.join(ROOT, 'models', 'impact_mlp.npz')}")
    print(f"roslaunch: {subprocess.run(['which', 'roslaunch'], capture_output=True, text=True).stdout.strip()}")


def run(cmd):
    print("+", " ".join(cmd))
    raise SystemExit(subprocess.call(cmd, cwd=ROOT))


def main():
    ap = argparse.ArgumentParser(description="四旋翼风扰物资精准空投")
    ap.add_argument("--info", action="store_true", help="显示环境/模型信息")
    ap.add_argument("--test", action="store_true", help="运行 15 个单元测试")
    ap.add_argument("--train", action="store_true", help="训练落点预测 MLP")
    ap.add_argument("--evaluate", action="store_true", help="运行 10 组随机风闭环评估")
    ap.add_argument("--record", action="store_true", help="录制 Ubuntu 桌面 demo")
    ap.add_argument("--launch", choices=["demo", "headless", "teleop"], help="启动 ROS launch")
    ap.add_argument("--wind-x", type=float, default=3.0)
    ap.add_argument("--wind-y", type=float, default=1.0)
    args = ap.parse_args()

    if args.test or args.train or args.evaluate:
        ensure_venv()

    if args.info:
        info()
    elif args.test:
        run([sys.executable, os.path.join(ROOT, "tests", "run_all.py")])
    elif args.train:
        run([sys.executable, os.path.join(ROOT, "tools", "train_predictor.py")])
    elif args.evaluate:
        run([sys.executable, os.path.join(ROOT, "tools", "evaluate.py")])
    elif args.record:
        run(["bash", os.path.join(ROOT, "scripts", "record_demo.sh")])
    elif args.launch:
        cmd = ["roslaunch", "uav_wind_aware_delivery", f"{args.launch}.launch"]
        if args.launch != "teleop":
            cmd += [f"wind_x:={args.wind_x}", f"wind_y:={args.wind_y}"]
        raise SystemExit(subprocess.call(cmd))
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
