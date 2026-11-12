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


def test_summarize_counts_missing_by_seq_range():
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
    # 乱序不等于丢包：0/1/2 全部到达，missing 必须为 0
    assert summary['missing'] == 0
    assert summary['expected'] == 3


def test_reordering_does_not_inflate_missing():
    """按 gap 累加会把乱序报文重复计为丢失，去了重之后不应虚高。"""
    seqs = (1, 2, 3, 6, 5, 7)
    rows = [_row(seqs[0], 1_000_000_000, gap=seqs[0] + 1)]
    for prev, cur in zip(seqs, seqs[1:]):
        rows.append(_row(cur, 1_000_000_000 + 5_000_000 * len(rows),
                         gap=cur - prev))
    summary = summarize(rows)
    assert summary['missing'] == 2  # 只缺 0 与 4
    assert summary['expected'] == 8
    assert summary['out_of_order'] == 1


def test_summarize_with_expected_msgs_counts_tail_loss():
    """给了 expected_msgs，末尾丢包与发布端欠触发都计入 missing。"""
    rows = [_row(0, 1_000_000_000), _row(1, 1_005_000_000),
            _row(2, 1_010_000_000)]
    summary = summarize(rows, expected_msgs=5)
    assert summary['expected'] == 5
    assert summary['missing'] == 2  # 3、4 未到达（末尾缺口）
    assert summary['drop_rate'] == pytest.approx(0.4)


def test_summarize_empty_with_expected_is_full_loss():
    summary = summarize([], expected_msgs=10)
    assert summary['count'] == 0
    assert summary['missing'] == 10
    assert summary['drop_rate'] == pytest.approx(1.0)


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
