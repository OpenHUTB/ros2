#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""兼容入口（薄封装，不重复实现控制器）。

评审要求模块只保留一份 PID 实现。唯一实现位于
``cruise_pid_node.py``；本文件仅做转发复用，便于沿用历史脚本名调用。
"""

from cruise_pid_node import (  # noqa: F401
    PIDCruiseController,
    UGVPlant,
    evaluate,
    main,
    simulate,
)

if __name__ == "__main__":
    main()
