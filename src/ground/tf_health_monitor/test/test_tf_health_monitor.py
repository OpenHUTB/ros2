#!/usr/bin/env python3
"""tf_health_monitor 判据的单元测试。

全部针对 core 中的纯逻辑，不依赖 rclpy，因此没有 ROS 环境的机器也能直接
``python3 -m pytest test -v`` 回归。节点行为（话题收发）见文档的演示步骤。
"""

import pytest

from tf_health_monitor import core


def feed(tracker, parent, child, count, rate=10.0, stamp_step=None,
         start=0.0, start_stamp=0, static=False):
    """按固定频率喂观测，返回最后一次观测的时刻。

    默认让 header.stamp 与到达时刻同步推进，避免误触「消息陈旧」判据。
    """
    step = int(1e9 / rate) if stamp_step is None else int(stamp_step)
    now = start
    stamp = int(start_stamp)
    for index in range(count):
        tracker.observe(parent, child, stamp, now, static=static)
        now += 1.0 / rate
        stamp += step
    return now - 1.0 / rate


def only(tracker, now):
    """取唯一一条边的判定结果。"""
    reports = tracker.evaluate(now)
    assert len(reports) == 1
    return reports[0]


# ---------------------------------------------------------------- 监视项解析

def test_parse_watched_frames_basic():
    assert core.parse_watched_frames(['map->odom']) == [('map', 'odom')]


def test_parse_watched_frames_tolerates_spaces():
    assert core.parse_watched_frames(
        [' odom -> base_footprint ']) == [('odom', 'base_footprint')]


def test_parse_watched_frames_deduplicates():
    assert core.parse_watched_frames(
        ['map->odom', 'map->odom']) == [('map', 'odom')]


def test_parse_watched_frames_strips_leading_slash():
    assert core.parse_watched_frames(['/map->/odom']) == [('map', 'odom')]


def test_parse_watched_frames_empty_means_discover_all():
    assert core.parse_watched_frames([]) == []
    assert core.parse_watched_frames(['', '  ']) == []


def test_parse_watched_frames_rejects_missing_arrow():
    with pytest.raises(ValueError):
        core.parse_watched_frames(['map odom'])


def test_parse_watched_frames_rejects_same_frame():
    with pytest.raises(ValueError):
        core.parse_watched_frames(['base_link->base_link'])


# ------------------------------------------------------------ 正常 / 断链

def test_fresh_edge_is_ok():
    tracker = core.TfHealthTracker()
    now = feed(tracker, 'map', 'odom', 10)
    report = only(tracker, now)
    assert report.status == core.OK
    assert report.rate == pytest.approx(10.0, rel=0.05)


def test_stale_after_timeout():
    tracker = core.TfHealthTracker(stale_timeout=0.5)
    now = feed(tracker, 'map', 'odom', 10)
    report = only(tracker, now + 1.0)
    assert report.status == core.STALE
    assert report.age == pytest.approx(1.0, abs=0.02)


def test_stale_boundary_is_inclusive_at_timeout():
    tracker = core.TfHealthTracker(stale_timeout=0.5)
    now = feed(tracker, 'map', 'odom', 10)
    assert only(tracker, now + 0.49).status == core.OK
    assert only(tracker, now + 0.51).status == core.STALE


def test_stale_recovery():
    tracker = core.TfHealthTracker(stale_timeout=0.5)
    now = feed(tracker, 'map', 'odom', 10)
    assert only(tracker, now + 1.0).status == core.STALE
    feed(tracker, 'map', 'odom', 6, start=now + 1.1, start_stamp=2e9)
    assert only(tracker, now + 1.6).status == core.OK


# ------------------------------------------------------------ 时间戳回退

def test_backwards_detected_and_latched():
    tracker = core.TfHealthTracker(stale_timeout=0.5, backwards_hold=2.0)
    tracker.observe('map', 'odom', 100, 0.0)
    tracker.observe('map', 'odom', 50, 0.1)
    report = only(tracker, 0.2)
    assert report.status == core.BACKWARDS
    assert report.backwards_count == 1


def test_backwards_hold_expires():
    tracker = core.TfHealthTracker(
        stale_timeout=5.0, min_samples=3, backwards_hold=0.5)
    # 10 Hz 下正常递增，第 5 帧的时间戳回落 0.05 s。
    stamps = [0, 100000000, 200000000, 300000000, 250000000, 400000000]
    for index, stamp in enumerate(stamps):
        tracker.observe('map', 'odom', stamp, index * 0.1)
    assert only(tracker, 0.5).status == core.BACKWARDS
    assert only(tracker, 1.2).status == core.OK


def test_backwards_does_not_count_repeated_forward_stamps():
    tracker = core.TfHealthTracker()
    for index in range(5):
        tracker.observe('map', 'odom', 100, index * 0.1)
    report = only(tracker, 0.4)
    assert report.backwards_count == 0
    assert report.status == core.OK


# -------------------------------------------------------------- 多 parent

def test_multi_parent_detected():
    tracker = core.TfHealthTracker(stale_timeout=0.5)
    tracker.observe('map', 'base_link', 0, 0.0)
    tracker.observe('world', 'base_link', 0, 0.0)
    reports = tracker.evaluate(0.0)
    assert [item.status for item in reports] == [core.MULTI_PARENT] * 2
    assert {item.parent for item in reports} == {'map', 'world'}


def test_multi_parent_clears_when_one_side_goes_stale():
    tracker = core.TfHealthTracker(stale_timeout=0.5)
    tracker.observe('world', 'base_link', 0, 0.0)
    now = feed(tracker, 'map', 'base_link', 8, start=0.0)
    report = [item for item in tracker.evaluate(now)
              if item.parent == 'map'][0]
    assert report.status == core.OK


# --------------------------------------------------------- 频率与样本数量

def test_low_rate_detected():
    tracker = core.TfHealthTracker(
        stale_timeout=5.0, rate_window=2.0, min_rate=10.0, min_samples=3)
    now = feed(tracker, 'map', 'odom', 7, rate=2.0)
    report = only(tracker, now)
    assert report.status == core.LOW_RATE
    assert report.rate == pytest.approx(2.0, rel=0.05)


def test_unknown_when_samples_insufficient():
    tracker = core.TfHealthTracker(min_samples=5)
    now = feed(tracker, 'map', 'odom', 2)
    assert only(tracker, now).status == core.UNKNOWN


# -------------------------------------------------------------- 消息陈旧

def test_old_data_detected():
    tracker = core.TfHealthTracker(stale_timeout=5.0, stamp_lag=1.0)
    for index in range(8):
        now = index * 0.1
        # 消息在按时到达，但时间戳固定落后 2 s。
        tracker.observe('map', 'odom', int((now - 2.0) * 1e9), now)
    report = only(tracker, 0.7)
    assert report.status == core.OLD_DATA
    assert report.skew == pytest.approx(2.0, abs=0.01)


def test_zero_stamp_is_not_treated_as_old_data():
    tracker = core.TfHealthTracker(stale_timeout=5.0, stamp_lag=1.0)
    for index in range(8):
        tracker.observe('map', 'odom', 0, index * 0.1)
    assert only(tracker, 0.7).status == core.OK


def test_static_edge_ignores_old_data():
    tracker = core.TfHealthTracker(stale_timeout=0.5, stamp_lag=1.0)
    tracker.observe('base_link', 'laser', 0, 0.0, static=True)
    assert only(tracker, 3.0).status == core.OK


# ------------------------------------------------------------ 声明但缺失

def test_watched_but_missing():
    tracker = core.TfHealthTracker(watched=[('map', 'odom')])
    reports = tracker.evaluate(0.0)
    assert len(reports) == 1
    assert reports[0].status == core.MISSING
    assert reports[0].age == float('inf')


def test_watched_edge_appears_once_it_arrives():
    tracker = core.TfHealthTracker(watched=[('map', 'odom')])
    now = feed(tracker, 'map', 'odom', 10)
    assert only(tracker, now).status == core.OK


# -------------------------------------------------------------- 静态变换

def test_static_edge_never_goes_stale():
    tracker = core.TfHealthTracker(stale_timeout=0.5)
    tracker.observe('base_link', 'laser', 0, 0.0, static=True)
    report = only(tracker, 10.0)
    assert report.status == core.OK
    assert report.static is True


def test_static_edge_still_detects_backwards():
    tracker = core.TfHealthTracker(backwards_hold=2.0)
    tracker.observe('base_link', 'laser', 100, 0.0, static=True)
    tracker.observe('base_link', 'laser', 50, 0.1, static=True)
    assert only(tracker, 0.2).status == core.BACKWARDS


def test_static_flag_survives_later_dynamic_observations():
    tracker = core.TfHealthTracker(stale_timeout=0.5)
    tracker.observe('base_link', 'laser', 0, 0.0, static=True)
    tracker.observe('base_link', 'laser', 1, 1.0)
    assert only(tracker, 5.0).status == core.OK


# ------------------------------------------------------ 斜杠写法归一化

def test_leading_slash_merges_into_single_edge():
    tracker = core.TfHealthTracker()
    tracker.observe('/map', '/odom', 0, 0.0)
    tracker.observe('map', 'odom', 1, 0.1)
    reports = tracker.evaluate(0.1)
    assert len(reports) == 1
    assert reports[0].label() == 'map->odom'
    assert tracker.total_samples == 2


# ------------------------------------------------------------ 总体状态

def test_overall_no_input_when_no_edges():
    status, problems = core.overall_status([])
    assert status == core.NO_INPUT
    assert problems == []


def test_overall_ignores_unknown_only():
    reports = [core.EdgeReport('map', 'odom', core.UNKNOWN)]
    status, problems = core.overall_status(reports)
    assert status == core.UNKNOWN
    assert problems == []


def test_overall_returns_worst_status():
    reports = [
        core.EdgeReport('a', 'b', core.OK),
        core.EdgeReport('c', 'd', core.STALE),
        core.EdgeReport('e', 'f', core.MULTI_PARENT),
        core.EdgeReport('g', 'h', core.LOW_RATE),
    ]
    status, problems = core.overall_status(reports)
    assert status == core.MULTI_PARENT
    assert len(problems) == 3


def test_overall_reports_all_ok():
    reports = [core.EdgeReport('map', 'odom', core.OK)]
    status, problems = core.overall_status(reports)
    assert status == core.OK
    assert problems == []


# ------------------------------------------------------------ 输出格式

def test_format_helpers():
    report = core.EdgeReport('map', 'odom', core.OK, rate=10.0)
    stale = core.EdgeReport('odom', 'base_link', core.STALE)
    assert core.format_edge(report) == 'map->odom:ok:10.00'
    assert core.format_report([report, stale]) == (
        'map->odom:ok:10.00;odom->base_link:stale:0.00')
    assert core.format_problems([report, stale]) == (
        'odom->base_link(stale)')
    assert core.format_problems([]) == ''


# ------------------------------------------------------------ 参数校验

def test_validate_parameters_accepts_defaults():
    core.validate_parameters(0.5, 2.0, 1.0, 5, 2.0, 1.0, 5.0)


@pytest.mark.parametrize('index,value', [
    (0, 0.0),
    (0, -1.0),
    (1, 0.0),
    (2, 0.0),
    (3, 1),
    (4, 0.0),
    (5, 0.0),
    (6, 0.0),
])
def test_validate_parameters_rejects(index, value):
    args = [0.5, 2.0, 1.0, 5, 2.0, 1.0, 5.0]
    args[index] = value
    with pytest.raises(ValueError):
        core.validate_parameters(*args)
