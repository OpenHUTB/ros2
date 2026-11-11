"""关节指令载荷的编解码。

载荷布局（std_msgs/Float64MultiArray.data，长度 9）::

    [seq, stamp_sec, stamp_nanosec, q1, q2, q3, q4, q5, q6]

- seq        递增序号，接收端据此识别丢包与乱序（ROS 2 的消息本体没有序号字段）
- stamp_*    发送时刻，直接取 ROS 时钟的纳秒值再拆成秒/纳秒两段，避免浮点精度损失
- q1..q6     六个关节目标角（rad），顺序为左髋、左膝、左踝、右髋、右膝、右踝

本模块不依赖 rclpy，便于不启动 ROS 也能做单元测试。
"""

JOINT_COUNT = 6
HEADER_LEN = 3
NSEC_PER_SEC = 1_000_000_000


def encode(seq, stamp_ns, positions):
    """把序号、时间戳与关节角打包成 Float64MultiArray.data。"""
    if len(positions) != JOINT_COUNT:
        raise ValueError(
            'positions must contain %d values, got %d' % (JOINT_COUNT, len(positions)))
    sec, nanosec = divmod(int(stamp_ns), NSEC_PER_SEC)
    return [float(seq), float(sec), float(nanosec)] + [float(p) for p in positions]


def decode(data):
    """返回 (seq, stamp_ns, positions)；载荷过短时抛 ValueError。"""
    if len(data) < HEADER_LEN + JOINT_COUNT:
        raise ValueError('payload too short: %d' % len(data))
    seq = int(round(data[0]))
    stamp_ns = int(round(data[1])) * NSEC_PER_SEC + int(round(data[2]))
    positions = [float(v) for v in data[HEADER_LEN:HEADER_LEN + JOINT_COUNT]]
    return seq, stamp_ns, positions
