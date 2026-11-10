"""统计函数的单元测试，用构造数据覆盖丢包、乱序与时延分位数。"""

import math

import pytest

from joint_qos_latency_eval.stats import percentile, summarize


def _row(seq, recv_ns, latency_ms=1.0, gap=1):
    return {
        'seq': seq,
        'stamp_ns': recv_ns - int(latency_ms * 1e6),
        'recv_ns': recv_ns,
        'latency_ms': latency_ms,
        'gap': gap,
    }


def test_percentile_interpolates():
    assert percentile([1.0], 50.0) == 1.0
    assert percentile([1.0, 2.0, 3.0, 4.0], 50.0) == pytest.approx(2.5)
    assert percentile([1.0, 2.0, 3.0, 4.0], 100.0) == 4.0


def test_percentile_of_empty_is_nan():
    assert math.isnan(percentile([], 50.0))


def test_percentile_rejects_out_of_range():
    with pytest.raises(ValueError):
        percentile([1.0, 2.0], 0.0)


def test_summarize_counts_missing_from_gaps():
    rows = [_row(0, 1_000_000_000), _row(1, 1_005_000_000),
            _row(4, 1_010_000_000, gap=3)]
    summary = summarize(rows)
    assert summary['count'] == 3
    assert summary['missing'] == 2
    assert summary['expected'] == 5
    assert summary['drop_rate'] == pytest.approx(0.4)


def test_summarize_detects_out_of_order():
    rows = [_row(0, 1_000_000_000), _row(2, 1_005_000_000),
            _row(1, 1_010_000_000, gap=-1)]
    summary = summarize(rows)
    assert summary['out_of_order'] == 1


def test_summarize_latency_extremes():
    rows = [_row(0, 1_000_000_000, latency_ms=1.0),
            _row(1, 1_005_000_000, latency_ms=2.0),
            _row(2, 1_010_000_000, latency_ms=3.0)]
    summary = summarize(rows)
    assert summary['latency_mean_ms'] == pytest.approx(2.0)
    assert summary['latency_max_ms'] == pytest.approx(3.0)
    assert summary['period_ms'] == pytest.approx(5.0)


def test_summarize_empty_returns_nan_metrics():
    summary = summarize([])
    assert summary['count'] == 0
    assert math.isnan(summary['latency_p99_ms'])
