"""状态机与事件对照的离线测试。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_visual_lane_departure_warning import lanes

CASES = []


def case(func):
    CASES.append(func)
    return func


@case
def starts_normal():
    """初始状态是 normal，没有偏移就没有事件。"""
    machine = lanes.LaneDepartureStateMachine()
    assert machine.state == lanes.NORMAL
    state, event = machine.update(0.0)
    assert state == lanes.NORMAL and event is None


@case
def deviation_then_departure():
    """越过两个阈值依次进入 deviation 和 departure，并带上方向。"""
    machine = lanes.LaneDepartureStateMachine()
    state, event = machine.update(0.6)
    assert state == lanes.DEVIATION and event == 'deviation_start_left'

    state, event = machine.update(1.0)
    assert state == lanes.DEPARTURE and event == 'departure_start_left'


@case
def right_side_events_use_right_suffix():
    """右偏时事件后缀是 right。"""
    machine = lanes.LaneDepartureStateMachine()
    _, event = machine.update(-0.6)
    assert event == 'deviation_start_right'


@case
def hysteresis_keeps_state_between_thresholds():
    """0.4 m 在 warn_enter(0.5) 与 warn_exit(0.3) 之间，不应退出 deviation。"""
    machine = lanes.LaneDepartureStateMachine()
    machine.update(0.6)
    state, event = machine.update(0.4)
    assert state == lanes.DEVIATION and event is None


@case
def recovery_walks_back_through_states():
    """从 departure 回 normal 要经过 deviation，事件顺序固定。"""
    machine = lanes.LaneDepartureStateMachine()
    events = []
    # 0.5 低于 depart_exit(0.65)，0.7 仍然高于它，还留在 departure
    for offset in (0.6, 1.0, 0.5, 0.2):
        _, event = machine.update(offset)
        events.append(event)
    assert events == ['deviation_start_left', 'departure_start_left',
                      'departure_end', 'deviation_clear'], events


@case
def invalid_thresholds_rejected():
    """阈值不满足递增关系时必须报错，避免把滞回写反。"""
    try:
        lanes.LaneDepartureStateMachine(warn_enter=0.3, warn_exit=0.5)
    except ValueError:
        return
    raise AssertionError('非法阈值没有报错')


@case
def events_are_recorded():
    """每次跃迁都记进 events，便于事后统计。"""
    machine = lanes.LaneDepartureStateMachine()
    machine.update(0.6)
    machine.update(0.1)
    assert len(machine.events) == 2
    assert machine.events[0][0] == 'deviation_start_left'


@case
def compare_events_counts_matches():
    """同类型事件在容差内算匹配，多出来的是误报和漏报。"""
    visual = [(5.0, 'deviation_start_left'), (9.0, 'departure_start_left'),
              (12.0, 'deviation_start_left')]
    truth = [(5.05, 'deviation_start_left'), (9.0, 'departure_start_left')]
    result = lanes.compare_events(visual, truth, tolerance=0.1)
    assert result['matched'] == 2, result
    assert result['false_alarm'] == 1, result
    assert result['missed'] == 0, result
    assert abs(result['mean_delay']) < 0.1


@case
def compare_events_reports_missed():
    """真值有、视觉没报的事件算漏报。"""
    visual = [(5.0, 'deviation_start_left')]
    truth = [(5.0, 'deviation_start_left'), (20.0, 'departure_start_right')]
    result = lanes.compare_events(visual, truth, tolerance=0.1)
    assert result['missed'] == 1, result
    assert result['matched'] == 1


def main():
    failures = 0
    for func in CASES:
        try:
            func()
            print('[通过] %s' % func.__name__)
        except AssertionError as error:
            failures += 1
            print('[失败] %s：%s' % (func.__name__, error))
    print('=' * 60)
    if failures:
        print('[结果] %d 项失败，%d 项通过' % (failures, len(CASES) - failures))
        return 1
    print('[结果] %d 项全部通过' % len(CASES))
    return 0


if __name__ == '__main__':
    sys.exit(main())