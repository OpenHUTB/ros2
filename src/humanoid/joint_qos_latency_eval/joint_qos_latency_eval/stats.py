"""纯函数统计，不依赖 rclpy，便于单独测试。

summarize() 接受的 rows 为按到达顺序排列的字典列表，键：
    seq         序号
    stamp_ns    发送时刻（发送端 ROS 时钟，纳秒）
    recv_ns     接收时刻（接收端 ROS 时钟，纳秒）
    latency_ms  单程时延（毫秒）
    gap         与上一条消息的序号差（1 表示连续）

expected_msgs 为发布端应发总条数（RATE × DURATION，由 run_sweep.sh / launch
传入）。不给（默认 0）时分母退化为 max(seq)+1，只看序号区间内的缺口；
给了之后末尾丢包和发布端欠触发也会计入 missing，口径更严格。
"""

import math


def percentile(values, q):
    """线性插值分位数，q 取 (0, 100]。空输入返回 nan。"""
    if not 0.0 < q <= 100.0:
        raise ValueError('percentile must be in (0, 100], got %r' % (q,))
    data = sorted(float(v) for v in values)
    if not data:
        return float('nan')
    pos = (len(data) - 1) * q / 100.0
    low = int(math.floor(pos))
    high = int(math.ceil(pos))
    return data[low] + (data[high] - data[low]) * (pos - low)


def summarize(rows, expected_msgs=0):
    count = len(rows)
    if count == 0:
        return {
            'count': 0,
            'missing': expected_msgs,
            'expected': expected_msgs,
            'drop_rate': (1.0 if expected_msgs else float('nan')),
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
    # 发布端序号从 0 递增，完整序列即 [0, max_seq]。这里按去重后的序号数算缺口，
    # 而不是累加相邻 gap：乱序到达（如 1,2,3,6,5,7）时同一个序号会被前后两条
    # 报文各算一次缺口，得出偏高的丢包率。
    seqs = [int(r['seq']) for r in rows]
    if expected_msgs > 0:
        # 以应发总数为分母：末尾丢包（max(seq) 之后的）和发布端欠触发都算进来
        expected = expected_msgs
    else:
        expected = max(seqs) + 1
    missing = max(0, expected - len(set(seqs)))
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
