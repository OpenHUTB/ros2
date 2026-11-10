"""实验用的 QoS 组合。

所有场景固定 history=KEEP_LAST，只改变可靠性、队列深度与持久性，
这样对比结果只反映 QoS 选择本身，而不是其他配置的叠加效应。
"""

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

DEFAULT_PROFILE = 'reliable_depth10'

_CONFIGS = {
    'reliable_depth10': {
        'reliability': ReliabilityPolicy.RELIABLE,
        'depth': 10,
    },
    'reliable_depth1': {
        'reliability': ReliabilityPolicy.RELIABLE,
        'depth': 1,
    },
    'best_effort_depth10': {
        'reliability': ReliabilityPolicy.BEST_EFFORT,
        'depth': 10,
    },
    'best_effort_depth1': {
        'reliability': ReliabilityPolicy.BEST_EFFORT,
        'depth': 1,
    },
    'reliable_transient_local': {
        'reliability': ReliabilityPolicy.RELIABLE,
        'depth': 1,
        'durability': DurabilityPolicy.TRANSIENT_LOCAL,
    },
}


def make(name):
    """按名称构造 QoSProfile；未知名称抛 KeyError 并列出可用项。"""
    try:
        config = dict(_CONFIGS[name])
    except KeyError:
        raise KeyError('unknown qos profile %r, available: %s'
                       % (name, ', '.join(_CONFIGS)))
    config.setdefault('history', HistoryPolicy.KEEP_LAST)
    return QoSProfile(**config)
