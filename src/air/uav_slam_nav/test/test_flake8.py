# Copyright 2026 OpenHUTB
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See http://www.apache.org/licenses/LICENSE-2.0 for the full text.
"""代码风格检查：整个包必须通过 flake8（行宽上限 99）。"""

import pytest
from ament_flake8.main import main_with_errors


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """跑一遍 flake8，有风格问题就失败并打印出来。"""
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, '发现 %d 处风格问题：\n%s' % (len(errors), '\n'.join(errors))
