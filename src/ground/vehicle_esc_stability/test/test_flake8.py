# Copyright 2026 李英杰
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""代码风格检查，只在 colcon test 下跑（`pytest tests` 不会收这个目录）。

配置用同目录的 flake8.ini：只查 pycodestyle / pyflakes 的 E、W、F 类，99 列。
ament 自带的 ament_flake8.ini 不会 select，本机若额外装了 flake8-docstrings、
flake8-quotes、flake8-import-order，就会冒出 D400 / D202 / Q000 / I100 这类
跟本仓库风格无关的告警，同一个提交在不同机器上结果不一致。自己带配置可以
把检查范围钉死，换成 select 而不是逐个 extend-ignore，插件再多也不会误报。
"""

import os

from ament_flake8.main import main_with_errors
import pytest

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'flake8.ini')


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """按 flake8.ini（E/W/F，99 列）检查代码风格。"""
    rc, errors = main_with_errors(argv=['--config=' + CONFIG])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
