#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""模块统一入口（规范要求入口以 main 开头）。

这里不承载业务逻辑，只是把三种常用玩法转发到 tools/ 下对应脚本，
好处是不做 ROS 启动的人也能 `python3 main.py` 直接看到效果：

    python3 main.py                # 离线闭环自检：MCL vs 纯里程计（只需 NumPy）
    python3 main.py --experiment   # 训练神经网络 + 四组对比 + 出图（需要 MuJoCo）
    python3 main.py --scene        # 由 src/lidar_mcl_localization/scene_layout.py 重新生成场景与地图

需要 ROS 时用 launch 入口：
    roslaunch lidar_mcl_localization main.launch gui:=true rviz:=true

用 subprocess 而不是 import，是为了让各脚本保持"可独立运行"的性质
（它们自己管理 sys.path，import 进同一个进程反而容易互相污染）。
"""

import argparse
import os
import subprocess
import sys

PKG_ROOT = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(PKG_ROOT, "tools")


def run(script, extra):
    """转发到 tools/<script>，透传额外参数，并原样返回退出码。"""
    path = os.path.join(TOOLS, script)
    if not os.path.isfile(path):
        print("找不到 %s —— 包目录是否完整？" % path, file=sys.stderr)
        return 1
    cmd = [sys.executable, path] + list(extra)
    print("[main.py] %s" % " ".join([os.path.basename(p) for p in cmd]))
    return subprocess.call(cmd, cwd=PKG_ROOT)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="lidar_mcl_localization 统一入口（默认跑离线闭环自检）")
    group = ap.add_mutually_exclusive_group()
    group.add_argument("--experiment", action="store_true",
                       help="训练神经网络 + 四组对比 + 出图（需要 MuJoCo）")
    group.add_argument("--scene", action="store_true",
                       help="重新生成 Gazebo 场景与占据栅格地图")
    ap.add_argument("rest", nargs=argparse.REMAINDER,
                    help="透传给底层脚本的参数，例如 -- --episodes 60")
    args = ap.parse_args(argv)

    extra = [a for a in args.rest if a != "--"]

    if args.experiment:
        return run("run_experiment.py", extra)
    if args.scene:
        return run("gen_scene.py", extra)
    return run("offline_demo.py", extra)


if __name__ == "__main__":
    sys.exit(main())
