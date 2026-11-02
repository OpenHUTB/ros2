#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_actuator_fault_sim 统一入口：argparse 解析各子命令并分发。

五个子命令分别对应不同的使用场景::

    --demo      不开 ROS，直接跑"健康 vs 6 种故障"对比表（最快的上手方式）
    --eval      跑完整矩阵（5 场景 × 7 故障）并写 results/comparison.csv
    --gif       录一张"健康 / 单桨失效"并排动图
    --keyboard  终端键盘直飞（任务①要求的遥操作，可叠加故障）
    --node      启动 ROS2 节点（也可 ros2 launch uav_actuator_fault_sim fault_sim.launch.py）
    --selftest  内置自检，提交 PR 前跑一遍

除 ``--node`` 外都不需要 ROS；除 ``--keyboard``/``--node`` 外都是无人值守的批处理。
"""

from __future__ import annotations

import argparse

from .scenario import FAULT_SCENARIOS, PHYSICAL_SCENES


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="main.py",
        description="uav_actuator_fault_sim —— 无人机执行器/传感器故障注入仿真层",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--demo", action="store_true", help="跑对比表（不用 ROS）")
    g.add_argument("--eval", metavar="OUT.csv", nargs="?", const="results/comparison.csv",
                   help="跑完整矩阵并写 CSV，默认 results/comparison.csv")
    g.add_argument("--gif", metavar="OUT.gif", help="录制故障对比动图（需要 Pillow）")
    g.add_argument("--keyboard", action="store_true", help="终端键盘直飞")
    g.add_argument("--node", action="store_true", help="启动 ROS2 节点")
    g.add_argument("--selftest", action="store_true", help="内置自检")

    ap.add_argument("--scene", default="open_ground", choices=PHYSICAL_SCENES,
                    help="物理场景（默认 open_ground）")
    ap.add_argument("--seed", type=int, default=0, help="随机种子")
    ap.add_argument("--fault", default="single_loss", choices=tuple(FAULT_SCENARIOS),
                    help="故障类型（--keyboard / --gif 用）")
    ap.add_argument("--rotor", type=int, default=0, help="故障桨序号 0~3")
    ap.add_argument("--onset", type=float, default=2.0, help="故障注入时刻（秒）")
    ap.add_argument("--severity", type=float, default=None, help="故障强度")
    ap.add_argument("--seconds", type=float, default=8.0, help="仿真时长（秒）")
    ap.add_argument("--width", type=int, default=320, help="录制分辨率宽")
    ap.add_argument("--height", type=int, default=240, help="录制分辨率高")
    ap.add_argument("--quality", choices=["fast", "full"], default="full",
                    help="渲染质量：full 好看、fast 快 6 倍")
    ap.add_argument("--fps", type=int, default=10, help="动图帧率")
    ap.add_argument("--camera", choices=["track", "fpv"], default="track",
                    help="录制视角：track 外部跟拍（默认，能看见姿态与坠地）、fpv 机载前视")
    ap.add_argument("--seeds", default="0", help="--eval 用：逗号分隔的种子，如 0,1,2")
    ap.add_argument("--teleop-gif", default=None, metavar="OUT.gif",
                    help="--keyboard 用：直飞时同时录制动图")
    ap.add_argument("--verbose", "-v", action="store_true")
    return ap


# --------------------------------------------------------------------------- #
def cmd_demo(a) -> int:
    from .evaluate import format_summary, format_table, run_comparison
    from .runner import RunConfig

    cfg = RunConfig(duration=a.seconds)
    results = run_comparison(scene=a.scene, seed=a.seed, onset=a.onset,
                             rotor=a.rotor, cfg=cfg)
    print(f"\n场景 {a.scene}（seed={a.seed}，桨{a.rotor}，{a.onset:.1f}s 注入，"
          f"{a.seconds:.0f}s 仿真）\n")
    print(format_table(results))
    print()
    print(format_summary(results))
    return 0


def cmd_eval(a) -> int:
    from .evaluate import format_summary, run_matrix, write_csv

    seeds = tuple(int(s) for s in str(a.seeds).split(",") if s.strip() != "")
    results = run_matrix(seeds=seeds)
    path = write_csv(results, a.eval)
    print(f"写出一共 {len(results)} 条记录到 {path}\n")
    print(format_summary(results))
    return 0


def cmd_gif(a) -> int:
    from .render import make_comparison_gif

    path = make_comparison_gif(
        a.gif, scene=a.scene, seed=a.seed, onset=a.onset, rotor=a.rotor,
        seconds=min(a.seconds, 8.0), width=a.width, height=a.height,
        fps=a.fps, quality=a.quality, camera=a.camera,
    )
    import os

    size_mb = os.path.getsize(path) / (1024 * 1024)
    print(f"已生成 {path}（{size_mb:.2f} MB）")
    if size_mb > 5.0:
        print("提示：超过 5 MB，建议降低 --width/--height 或 --fps 再录一次")
    return 0


def cmd_keyboard(a) -> int:
    from .keyboard_teleop import run_keyboard_teleop

    return run_keyboard_teleop(
        scene=a.scene, seed=a.seed, seconds=a.seconds,
        fault_scenario=a.fault, rotor=a.rotor, onset=a.onset,
        severity=a.severity, gif_path=a.teleop_gif,
        width=a.width, height=a.height, quality=a.quality,
    )


def cmd_node(_a) -> int:
    from .ros_node import main as node_main
    return node_main()


def cmd_selftest(a) -> int:
    from .selftest import run_selftest
    return run_selftest(verbose=True)


# --------------------------------------------------------------------------- #
def main(argv=None) -> int:
    a = build_parser().parse_args(argv)
    if a.demo:
        return cmd_demo(a)
    if a.eval is not None:
        return cmd_eval(a)
    if a.gif:
        return cmd_gif(a)
    if a.keyboard:
        return cmd_keyboard(a)
    if a.node:
        return cmd_node(a)
    if a.selftest:
        return cmd_selftest(a)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
