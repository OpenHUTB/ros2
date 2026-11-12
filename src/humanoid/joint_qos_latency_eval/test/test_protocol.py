"""载荷编解码的单元测试。"""

import pytest

from joint_qos_latency_eval.protocol import decode, encode


def test_roundtrip_preserves_seq_and_stamp():
    positions = [0.1, -0.2, 0.0, 0.3, 0.4, -0.5]
    data = encode(7, 1700000000123456789, positions)
    seq, stamp_ns, back = decode(data)
    assert seq == 7
    assert stamp_ns == 1700000000123456789
    assert back == pytest.approx(positions)


def test_nanosecond_precision_is_not_lost():
    """时间戳不能走浮点秒，否则 1e9 量级会丢精度。"""
    data = encode(0, 1700000000999999999, [0.0] * 6)
    _seq, stamp_ns, _ = decode(data)
    assert stamp_ns == 1700000000999999999


def test_wrong_joint_count_is_rejected():
    with pytest.raises(ValueError):
        encode(1, 0, [0.0] * 5)


def test_short_payload_is_rejected():
    with pytest.raises(ValueError):
        decode([1.0, 2.0, 3.0])
