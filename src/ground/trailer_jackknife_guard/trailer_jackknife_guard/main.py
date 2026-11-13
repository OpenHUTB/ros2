#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""入口：默认跑三个对照场景并打印报告；``--node`` 启动 ROS 2 节点。

    python3 -m trailer_jackknife_guard.main            # 离线报告
    python3 -m trailer_jackknife_guard.main --node     # ROS 2 节点
"""

from __future__ import annotations

import argparse
import sys

from .simulation import default_scenarios, format_report


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(description='半挂车折叠风险与限速保护')
    parser.add_argument('--node', action='store_true', help='启动 ROS 2 节点')
    parser.add_argument('--duration', type=float, default=30.0,
                        help='每个场景的仿真时长 [s]')
    args = parser.parse_args(argv)

    if args.node:
        from .node import main as node_main
        node_main([])
        return 0

    results = default_scenarios(duration=args.duration)
    print(format_report(results))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
