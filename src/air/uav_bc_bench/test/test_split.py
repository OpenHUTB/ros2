# -*- coding: utf-8 -*-
"""按圈分组切分的单元测试（不需要 ROS / 仿真器 / 数据文件）。

运行：python3 -m pytest test/test_split.py -q
"""

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from uav_bc_bench.bench import _split  # noqa: E402


def test_group_split_keeps_laps_together():
    groups = [0] * 200 + [1] * 100 + [2] * 100
    train, val, test, mode = _split(len(groups), 0, groups)
    assert mode == 'lap'
    assert min(test) >= 300, '测试集应来自最后一圈'
    assert len(set(train) & set(test)) == 0
    assert len(set(train) & set(val)) == 0
    assert len(set(val) & set(test)) == 0


def test_fallback_time_split_without_groups():
    train, val, test, mode = _split(100, 0, None)
    assert mode == 'time'
    assert len(train) + len(val) + len(test) == 100
    assert max(train) < min(val) < max(val) < min(test)
