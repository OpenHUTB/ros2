#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""作业五 · CARLA 整合 + 性能评价

把前四份作业统一入口，并提供性能评价指标。
  --target control/perception/navigation/end_to_end  启动对应子作业
  --benchmark                                        输出离线评价示例
  （默认）                                           打印帮助
"""
import argparse
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODULES = {
    "control":     os.path.join(ROOT, "01_control", "main.py"),
    "perception":  os.path.join(ROOT, "02_perception", "main.py"),
    "navigation":  os.path.join(ROOT, "03_navigation", "main.py"),
    "end_to_end":  os.path.join(ROOT, "04_end_to_end", "main.py"),
}


def run_module(target, extra):
    if target not in MODULES:
        print("[错误] 未知 target:", target); return 1
    cmd = [sys.executable, MODULES[target]] + extra
    print("启动:", target, "=>", " ".join(cmd))
    return subprocess.call(cmd)


def benchmark():
    """性能评价示例（文档数据来源；真实值需真车运行后填写）。"""
    import numpy as np
    np.random.seed(0)
    steer = np.clip(np.random.randn(200) * 0.12, -1, 1)
    speed = 9.0 + np.sin(np.linspace(0, 6, 200)) * 1.2
    lat_err = np.abs(np.random.randn(200)) * 0.2
    metrics = {
        "平均速度 (m/s)": round(float(speed.mean()), 2),
        "最大速度 (m/s)": round(float(speed.max()), 2),
        "转向均值": round(float(steer.mean()), 3),
        "转向标准差": round(float(steer.std()), 3),
        "转向变化量 AoS": round(float(np.abs(np.diff(steer)).mean()), 3),
        "横向误差 RMSE (m)": round(float(np.sqrt((lat_err ** 2).mean())), 3),
    }
    print("== 性能评价 ==")
    for k, v in metrics.items():
        print(f"  {k}: {v}")


def cli():
    p = argparse.ArgumentParser(description="CARLA 整合 + 性能评价")
    # choices 含空串：ROS launch 常传 --target ""（空=仅提示），需优雅接受
    p.add_argument("--target", choices=list(MODULES.keys()) + [""], default=None)
    p.add_argument("--benchmark", action="store_true")
    # 用 parse_known_args 把未知的 --xxx（如 --mode train）透传给子作业
    args, unknown = p.parse_known_args()
    extra = list(getattr(args, "extra", [])) + unknown
    if args.benchmark:
        benchmark()
    elif args.target:  # "" 为空则走 help
        run_module(args.target, extra)
    else:
        print("请指定 --target 或 --benchmark")
        p.print_help()


if __name__ == "__main__":
    cli()
