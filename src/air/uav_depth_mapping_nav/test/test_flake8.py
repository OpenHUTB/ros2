# Copyright 2026 OpenHUTB
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See http://www.apache.org/licenses/LICENSE-2.0 for the full text.
"""代码风格检查：整个包必须通过 flake8（行宽上限 99）。

`ament_flake8` 只存在于 ROS 2 的测试环境。没有它的时候（比如在普通 Python
环境里跑 `pytest test/`）用 `importorskip` 跳过，而不是让收集阶段直接
ImportError —— 否则别人 clone 下来跑一遍会看到一个和代码无关的失败。
"""

import os

import pytest

pytest.importorskip('ament_flake8')

from ament_flake8.main import main_with_errors  # noqa: E402


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """跑一遍 flake8，有风格问题就失败并打印出来。"""
    # ament_flake8 检查的是当前目录，测试可能在仓库根目录被调用，
    # 所以先切到包根，避免把别的包的风格问题算到本包头上。
    os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, '发现 %d 处风格问题：\n%s' % (len(errors), '\n'.join(errors))
