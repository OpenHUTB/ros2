# -*- coding: utf-8 -*-
"""避障决策逻辑的单元测试（纯函数，不需要 ROS 环境）。

运行方式（两种都行）：
    pytest src/carla_yolo_avoidance/test/test_decision_logic.py
    python src/carla_yolo_avoidance/test/test_decision_logic.py
"""

import os
import sys

_PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

from carla_yolo_avoidance import config, decision_logic as dl  # noqa: E402


# ---------------------------------------------------------------------------
# decide(): 距离 -> 控制量
# ---------------------------------------------------------------------------

def test_cruise_when_no_vehicle():
    """视野内没有车 -> 正常前进（踩油门、不刹车）。"""
    decision = dl.decide(min_distance_m=None, detection_age_s=0.1, has_detection=True)
    assert decision.state == 'CRUISE'
    assert decision.throttle == config.CRUISE_THROTTLE
    assert decision.brake == 0.0


def test_cruise_when_far_enough():
    """距离大于安全阈值 -> 正常前进。"""
    decision = dl.decide(min_distance_m=config.SAFE_DISTANCE_M + 5.0,
                         detection_age_s=0.1, has_detection=True)
    assert decision.state == 'CRUISE'
    assert decision.throttle == config.CRUISE_THROTTLE
    assert decision.brake == 0.0


def test_slow_down_below_safe_distance():
    """距离小于安全阈值 -> 必须刹车（作业核心要求）。"""
    decision = dl.decide(min_distance_m=config.SAFE_DISTANCE_M - 1.0,
                         detection_age_s=0.1, has_detection=True)
    assert decision.state == 'SLOW_DOWN'
    assert decision.brake > 0.0           # 有刹车
    assert decision.throttle == 0.0       # 同时不踩油门


def test_full_brake_when_very_close():
    """进入急刹区 -> 全力刹车。"""
    decision = dl.decide(min_distance_m=config.FULL_BRAKE_DISTANCE_M - 1.0,
                         detection_age_s=0.1, has_detection=True)
    assert decision.state == 'FULL_BRAKE'
    assert decision.brake == config.FULL_BRAKE
    assert decision.throttle == 0.0


def test_threshold_boundaries():
    """阈值边界：等于阈值算"安全"，严格小于才算越界。"""
    at_safe = dl.decide(config.SAFE_DISTANCE_M, 0.0, True)
    assert at_safe.state == 'CRUISE'

    just_inside = dl.decide(config.SAFE_DISTANCE_M - 0.001, 0.0, True)
    assert just_inside.state == 'SLOW_DOWN'

    at_full_brake = dl.decide(config.FULL_BRAKE_DISTANCE_M, 0.0, True)
    assert at_full_brake.state == 'SLOW_DOWN'

    just_inside_full_brake = dl.decide(config.FULL_BRAKE_DISTANCE_M - 0.001, 0.0, True)
    assert just_inside_full_brake.state == 'FULL_BRAKE'


def test_steer_is_always_zero():
    """作业要求只控制油门刹车，任何情况下 steer 都必须为 0。"""
    for distance in (None, 1.0, 8.0, 15.0, 20.0, 50.0):
        for age in (None, 0.0, 100.0):
            decision = dl.decide(distance, age, distance is not None)
            assert decision.steer == 0.0


def test_control_values_within_range():
    """控制量必须落在 [0, 1]，否则 CARLA 行为不可预期。"""
    for distance in (None, 0.5, 3.0, 10.0, 19.9, 20.0, 80.0):
        decision = dl.decide(distance, 0.0, distance is not None)
        assert 0.0 <= decision.throttle <= 1.0
        assert 0.0 <= decision.brake <= 1.0


def test_stale_detection_brakes():
    """检测结果超时 -> 失效安全：刹停而不是继续前进。"""
    stale = dl.decide(None, config.DETECTION_STALE_TIMEOUT_S + 1.0, True)
    assert stale.state == 'STALE_BRAKE'
    assert stale.brake == config.FULL_BRAKE
    assert stale.throttle == 0.0

    # 从未收到过检测结果也按失效处理
    never = dl.decide(None, None, False)
    assert never.state == 'STALE_BRAKE'


def test_stale_detection_does_not_override_when_disabled():
    """把"超时刹停"关掉时应改为滑行（验证开关有效）。"""
    original = config.BRAKE_WHEN_DETECTION_STALE
    config.BRAKE_WHEN_DETECTION_STALE = False
    try:
        decision = dl.decide(None, config.DETECTION_STALE_TIMEOUT_S + 1.0, True)
        assert decision.state == 'STALE_COAST'
        assert decision.brake == 0.0
    finally:
        config.BRAKE_WHEN_DETECTION_STALE = original


def test_is_detection_stale():
    assert dl.is_detection_stale(None) is True
    assert dl.is_detection_stale(0.0) is False
    assert dl.is_detection_stale(config.DETECTION_STALE_TIMEOUT_S) is False
    assert dl.is_detection_stale(config.DETECTION_STALE_TIMEOUT_S + 0.01) is True


def test_as_list_layout():
    """发布给 CARLA 的数组顺序固定为 [油门, 刹车, 转向]。"""
    decision = dl.decide(15.0, 0.0, True)
    values = decision.as_list()
    assert len(values) == config.CONTROL_CMD_SIZE
    assert values[0] == decision.throttle
    assert values[1] == decision.brake
    assert values[2] == decision.steer


# ---------------------------------------------------------------------------
# select_nearest_in_lane(): 挑出本车道最近的车辆
# ---------------------------------------------------------------------------

def _det(distance, lateral, name='car'):
    return {'class': name, 'distance_m': distance, 'lateral_m': lateral}


def test_select_nearest_picks_smallest_distance():
    detections = [_det(40.0, 0.0), _det(18.0, 0.5), _det(25.0, -0.5)]
    distance, nearest = dl.select_nearest_in_lane(detections)
    assert distance == 18.0
    assert nearest['distance_m'] == 18.0


def test_select_nearest_ignores_other_lane():
    """旁边车道的近车不应触发刹车。"""
    detections = [_det(5.0, 8.0), _det(30.0, 0.2)]  # 8 m 横向偏移属于旁边车道
    distance, _ = dl.select_nearest_in_lane(detections)
    assert distance == 30.0


def test_select_nearest_ignores_unreliable_distance():
    detections = [_det(None, 0.0), _det(22.0, 0.1)]
    distance, _ = dl.select_nearest_in_lane(detections)
    assert distance == 22.0


def test_select_nearest_empty():
    assert dl.select_nearest_in_lane([]) == (None, None)


def test_select_nearest_custom_lateral_limit():
    detections = [_det(6.0, 4.0)]
    assert dl.select_nearest_in_lane(detections)[0] is None
    assert dl.select_nearest_in_lane(detections, lateral_limit_m=5.0)[0] == 6.0


if __name__ == '__main__':
    failures = 0
    total = 0
    for _name in sorted(list(globals())):
        if _name.startswith('test_') and callable(globals()[_name]):
            total += 1
            try:
                globals()[_name]()
                print('PASS {}'.format(_name))
            except AssertionError as exc:
                failures += 1
                print('FAIL {} -> {}'.format(_name, exc))
    print('---- 通过 {} 个，失败 {} 个 ----'.format(total - failures, failures))
    sys.exit(1 if failures else 0)
