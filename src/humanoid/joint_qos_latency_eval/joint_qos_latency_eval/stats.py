"""纯函数统计，不依赖 rclpy，便于单独测试。

summarize() 接受的 rows 为按到达顺序排列的字典列表，键：
    seq         序号
    stamp_ns    发送时刻（发送端 ROS 时钟，纳秒）
    recv_ns     接收时刻（接收端 ROS 时钟，纳秒）
    latency_ms  单程时延（毫秒）
    gap         与上一条消息的序号差（1 表示连续）
"""

import math


def percentile(values, q):
    """线性插值分位数，q 取 (0, 100]。空输入返回 nan。"""
    data = sorted(float(v) for v in values)
    if not data:
        return float('nan')
    if not 0.0 < q <= 100.0:
        raise ValueError('percentile must be in (0, 100], got %r' % (q,))
    pos = (len(data) - 1) * q / 100.0
    low = int(math.floor(pos))
    high = int(math.ceil(pos))
    return data[low] + (data[high] - data[low]) * (pos - low)


def summarize(rows):
    count = len(rows)
    if count == 0:
        return {
            'count': 0,
            'missing': 0,
            'expected': 0,
            'drop_rate': float('nan'),
            'out_of_order': 0,
            'latency_mean_ms': float('nan'),
            'latency_p50_ms': float('nan'),
            'latency_p95_ms': float('nan'),
            'latency_p99_ms': float('nan'),
            'latency_max_ms': float('nan'),
            'jitter_ms': float('nan'),
            'period_ms': float('nan'),
        }

    latency = [float(r['latency_ms']) for r in rows]
    missing = sum(max(0, int(r['gap']) - 1) for r in rows)
    expected = count + missing
    out_of_order = sum(1 for prev, cur in zip(rows, rows[1:])
                       if int(cur['seq']) < int(prev['seq']))

    recv = [int(r['recv_ns']) for r in rows]
    deltas = [(later - earlier) / 1e6 for earlier, later in zip(recv, recv[1:])]
    if len(deltas) >= 2:
        period = sum(deltas) / len(deltas)
        jitter = math.sqrt(
            sum((d - period) ** 2 for d in deltas) / (len(deltas) - 1))
    else:
        period = float('nan')
        jitter = float('nan')

    return {
        'count': count,
        'missing': missing,
        'expected': expected,
        'drop_rate': (missing / expected) if expected else 0.0,
        'out_of_order': out_of_order,
        'latency_mean_ms': sum(latency) / count,
        'latency_p50_ms': percentile(latency, 50.0),
        'latency_p95_ms': percentile(latency, 95.0),
        'latency_p99_ms': percentile(latency, 99.0),
        'latency_max_ms': max(latency),
        'jitter_ms': jitter,
        'period_ms': period,
    }
