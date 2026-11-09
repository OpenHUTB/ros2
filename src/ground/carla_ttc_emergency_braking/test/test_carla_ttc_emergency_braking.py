"""AEB 判据的单元测试（不需要 CARLA 服务端）。

覆盖 app.should_brake 的三条边界：距离触发、TTC 触发、无目标/低速。
"""
import importlib.util
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APP = os.path.join(ROOT, 'carla_ttc_emergency_braking', 'app.py')


@pytest.fixture(scope='module')
def should_brake():
    """只加载 app.py 里的纯函数，避免 import carla。"""
    src = open(APP, encoding='utf-8').read()
    # 只取 should_brake 定义，剪掉对 carla 的依赖
    start = src.index('def should_brake(')
    end = src.index('def radar_front_target(')
    ns = {}
    exec(compile(src[start:end], APP, 'exec'), ns)  # noqa: S102
    return ns['should_brake']


def test_distance_triggers_brake(should_brake):
    brake, _ttc = should_brake(distance=20.0, speed=10.0, min_brake_distance=22.0, ttc_threshold=1.6)
    assert brake is True


def test_ttc_triggers_brake(should_brake):
    # 30 m @ 20 m/s -> TTC 1.5 s，距离没到 22 m 但 TTC 到阈值
    brake, ttc = should_brake(distance=30.0, speed=20.0, min_brake_distance=22.0, ttc_threshold=1.6)
    assert brake is True
    assert ttc == pytest.approx(1.5)


def test_far_target_does_not_brake(should_brake):
    brake, ttc = should_brake(distance=80.0, speed=10.0, min_brake_distance=22.0, ttc_threshold=1.6)
    assert brake is False
    assert ttc == pytest.approx(8.0)


def test_no_target_does_not_brake(should_brake):
    brake, ttc = should_brake(None, 10.0, 22.0, 1.6)
    assert brake is False and ttc == -1.0


def test_standstill_has_no_ttc(should_brake):
    brake, ttc = should_brake(distance=40.0, speed=0.0, min_brake_distance=22.0, ttc_threshold=1.6)
    assert brake is False and ttc == -1.0
