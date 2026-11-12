#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""入口：默认打印四个会遇场景的对照报告；``--node`` 启动 ROS 2 节点。"""

from __future__ import annotations

import argparse
import sys

from .simulation import default_scenarios, format_report


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(description='USV COLREGs 会遇判定与避让')
    parser.add_argument('--node', action='store_true', help='启动 ROS 2 节点')
    args = parser.parse_args(argv)

    if args.node:
        from .node import main as node_main
        node_main([])
        return 0

    print(format_report(default_scenarios()))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
