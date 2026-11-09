"""航点到点误差与配置的单元测试（不需要 AirSim 服务端）。"""
import os

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APP = os.path.join(ROOT, 'airsim_multirotor_waypoint_nav', 'app.py')


@pytest.fixture(scope='module')
def planar_error():
    src = open(APP, encoding='utf-8').read()
    start = src.index('def planar_error(')
    end = src.index('def position_of(')
    ns = {'math': __import__('math')}
    exec(compile(src[start:end], APP, 'exec'), ns)  # noqa: S102
    return ns['planar_error']


def test_error_is_zero_on_target(planar_error):
    assert planar_error((25.0, 25.0, -6.0), (25.0, 25.0)) == pytest.approx(0.0)


def test_error_is_planar_distance(planar_error):
    # NED 的 z 不参与平面误差
    assert planar_error((3.0, 4.0, -99.0), (0.0, 0.0)) == pytest.approx(5.0)


def test_error_ignores_altitude(planar_error):
    assert planar_error((0.0, 0.0, -6.0), (0.0, 0.0)) == pytest.approx(0.0)


def test_waypoints_config_is_4_points():
    with open(os.path.join(ROOT, 'config', 'params.yaml'), encoding='utf-8') as fh:
        cfg = yaml.safe_load(fh)
    wps = cfg['flight']['waypoints']
    assert len(wps) == 4
    for w in wps:
        assert len(w) == 2


def test_arrive_tolerance_is_positive():
    with open(os.path.join(ROOT, 'config', 'params.yaml'), encoding='utf-8') as fh:
        cfg = yaml.safe_load(fh)
    assert cfg['flight']['arrive_tolerance'] > 0
