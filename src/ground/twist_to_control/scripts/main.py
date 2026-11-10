#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""twist_to_control 的启动入口（仓库约定的 main.* 形式）。

所有逻辑都在 twist_node.py 里，这里只准备模块搜索路径并调用它，
这样 launch 文件可以统一用 type="main.py"，与其他模块保持一致。
"""
from __future__ import print_function

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main():
    from twist_node import main as node_main
    node_main()


if __name__ == "__main__":
    main()
