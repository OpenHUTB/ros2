#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""兼容入口：早期版本从这里直接运行，实现已迁到 cruise_pid_node.py。

保留本文件是为了不打断已有的引用路径；实际逻辑全部在同目录的
``cruise_pid_node.py``，本文件只做转发。
"""
import os
import runpy

HERE = os.path.dirname(os.path.abspath(__file__))

if __name__ == "__main__":
    runpy.run_path(os.path.join(HERE, "cruise_pid_node.py"), run_name="__main__")
