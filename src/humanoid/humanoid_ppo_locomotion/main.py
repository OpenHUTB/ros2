#!/usr/bin/env python3
"""统一入口：训练 / 演示 / ROS 2 部署。

课程要求入口以 main 开头，本文件即模块入口。

    python main.py --train --timesteps 3000000
    python main.py --play --weights weights/policy_g1.pt
    python main.py --deploy            # 等价于 ros2 launch humanoid_ppo_locomotion main.launch.py
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(HERE, "scripts")
sys.path.insert(0, HERE)


def _run(script: str, extra: list[str]) -> int:
    return subprocess.call([sys.executable, os.path.join(SCRIPTS, script), *extra])


def main() -> int:
    # 用 parse_known_args：模式开关之外的参数一律原样透传给底层脚本
    # （argparse 的 REMAINDER 遇到 "--weights" 这类选项会误判，故不用）
    ap = argparse.ArgumentParser(description="humanoid_ppo_locomotion 统一入口")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--train", action="store_true", help="训练 PPO 策略")
    mode.add_argument("--play", action="store_true", help="加载权重离线演示")
    mode.add_argument("--gif", action="store_true", help="录制演示动图")
    mode.add_argument("--eval", action="store_true", help="评价策略指标")
    mode.add_argument("--export", action="store_true", help="导出 TorchScript")
    mode.add_argument("--deploy", action="store_true", help="用 ros2 launch 启动完整链路")
    args, rest = ap.parse_known_args()

    rest = [a for a in rest if a != "--"]

    if args.train:
        return _run("train_ppo.py", rest)
    if args.play:
        return _run("play_policy.py", rest)
    if args.gif:
        return _run("record_gif.py", rest)
    if args.eval:
        return _run("evaluate.py", rest)
    if args.export:
        return _run("export_policy.py", rest)
    if args.deploy:
        cmd = ["ros2", "launch", "humanoid_ppo_locomotion", "main.launch.py", *rest]
        print("[main] " + " ".join(cmd))
        return subprocess.call(cmd)

    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
