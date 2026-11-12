#!/usr/bin/env python3
"""模块入口：一条命令跑通离线复现，不需要 ROS 与任何仿真器。

    python main.py                     # 7 个场景 × 3 个随机种子，并生成配图
    python main.py --seeds 5           # 多用几个种子
    python main.py --scenario slip     # 只跑单个场景
    python main.py --no-plots          # 只看指标表

输出：
  * 控制台打印"关掉脱困 vs 开启脱困"的对照指标；
  * 图片写入 docs_figs/（traces/detection_signals/state_timeline/compare/sweep）。

在线运行时改用 ROS2 节点：``ros2 launch ros2_stuck_recovery recovery.launch.py``。
"""
from __future__ import annotations

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent
for path in (str(ROOT / "src"), str(ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)

from sim.run_sim import main as run_offline  # noqa: E402


def main() -> None:
    run_offline()


if __name__ == "__main__":
    main()
