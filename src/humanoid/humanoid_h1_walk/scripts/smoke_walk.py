#!/usr/bin/env python3
"""无头自检：两个机型各跑一遍完整动作序列，检查「不摔倒 + 真的走了」。

不打开任何窗口，只跑物理与策略，约 30 s 跑完（31 s 仿真时间 × 2 机型）。

    python scripts/smoke_walk.py
    python scripts/smoke_walk.py --robot h1_2

判定标准（与上游 humanoid-walk 的 test_walk.py 一致）：

* 骨盆最低点 > 0.5 m —— 没有摔倒；
* 前进阶段位移 > 1.0 m —— 确实在走，不是原地踏步。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from humanoid_h1_walk import ROUTINE, Walker, load_specs, routine_command  # noqa: E402

MIN_PELVIS_HEIGHT = 0.5  # 米
MIN_TRAVEL = 1.0  # 米


def run_one(robot: str, specs) -> dict:
    walker = Walker(robot, specs)
    total = sum(seconds for _, seconds, _, _ in ROUTINE)
    lowest = 9.9
    start = None
    while walker.data.time < total:
        cmd, label = routine_command(walker.data.time)
        if label == "前进" and start is None:
            start = walker.data.qpos[:2].copy()
        walker.step(cmd)
        lowest = min(lowest, walker.base_height)

    travelled = float(np.linalg.norm(walker.data.qpos[:2] - start)) if start is not None else 0.0
    return {
        "robot": robot,
        "lowest": lowest,
        "travelled": travelled,
        "final_height": walker.base_height,
        "final_xy": walker.base_position[:2],
        "yaw": walker.yaw,
        "sim_time": float(walker.data.time),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--robot", action="append", choices=["h1", "h1_2"], help="只测指定机型，可重复")
    args = parser.parse_args()

    specs = load_specs()
    robots = args.robot or list(specs)
    results = [run_one(r, specs) for r in robots]

    print()
    print(f"{'机型':<6}{'最低骨盆':>10}{'前进位移':>10}{'末态高度':>10}{'偏航角':>9}")
    print("-" * 45)
    for r in results:
        print(
            f"{r['robot']:<6}{r['lowest']:>9.2f}m{r['travelled']:>9.2f}m"
            f"{r['final_height']:>9.2f}m{r['yaw']:>8.2f}r"
        )
    print()

    ok = True
    for r in results:
        if not np.isfinite(r["lowest"]) or not np.isfinite(r["travelled"]):
            print(f"[FAIL] {r['robot']}: 仿真发散（出现 NaN）")
            ok = False
        elif r["lowest"] <= MIN_PELVIS_HEIGHT:
            print(f"[FAIL] {r['robot']}: 骨盆最低 {r['lowest']:.2f} m ≤ {MIN_PELVIS_HEIGHT} m，摔倒了")
            ok = False
        elif r["travelled"] <= MIN_TRAVEL:
            print(f"[FAIL] {r['robot']}: 前进位移 {r['travelled']:.2f} m ≤ {MIN_TRAVEL} m，没有走起来")
            ok = False
        else:
            print(f"[PASS] {r['robot']}: 走完 {r['travelled']:.2f} m，未摔倒")
    print()
    print("ALL PASS" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
