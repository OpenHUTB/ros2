"""ROS 2 节点：订阅原始 CAN 帧，解码成物理量并发布校验结论。

话题
----
- 订阅 ``/can/frame``（``std_msgs/UInt8MultiArray``）：前 4 字节为帧 ID（小端 uint32），
  紧跟 8 字节载荷，共 12 字节。
- 发布 ``/can/signals``（``std_msgs/Float64MultiArray``）：按信号表顺序的物理量。
- 发布 ``/can/decode_status``（``std_msgs/String``）：CRC / 计数器 / 越界的结论。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, UInt8MultiArray

from .database import FrameCodec, default_frames
from .frame import MESSAGE_BYTES, unpack_message


class CanDecodeNode(Node):
    """CAN 帧解码节点。"""

    def __init__(self):
        super().__init__('can_signal_decode')
        self.declare_parameter('require_crc', True)
        self.declare_parameter('publish_invalid', True)

        self.require_crc = bool(self.get_parameter('require_crc').value)
        self.publish_invalid = bool(self.get_parameter('publish_invalid').value)

        self.codecs = {spec.frame_id: FrameCodec(spec, counter_name='AliveCounter')
                       for spec in default_frames()}
        self.previous = {}
        self.accepted = 0
        self.rejected = 0

        self.subscription = self.create_subscription(
            UInt8MultiArray, '/can/frame', self.on_frame, 50)
        self.signal_pub = self.create_publisher(Float64MultiArray, '/can/signals', 10)
        self.status_pub = self.create_publisher(String, '/can/decode_status', 10)

        self.get_logger().info(
            f'CAN 解码节点已启动：识别 {len(self.codecs)} 条报文 '
            f'({", ".join(f"0x{codec:03X}" for codec in sorted(self.codecs))})')

    def on_frame(self, msg):
        unpacked = unpack_message(list(msg.data))
        if unpacked is None:
            self.rejected += 1
            self.get_logger().warning(
                f'报文长度 {len(msg.data)} != {MESSAGE_BYTES}，已丢弃',
                throttle_duration_sec=2.0)
            return
        frame_id, payload = unpacked

        codec = self.codecs.get(frame_id)
        if codec is None:
            self.rejected += 1
            self.get_logger().debug(f'未知帧 ID 0x{frame_id:03X}')
            return

        decoded = codec.decode(payload, previous=self.previous.get(frame_id))
        self.previous[frame_id] = payload

        if not decoded['crc_ok'] and self.require_crc:
            self.rejected += 1
            self._publish_status(codec, decoded)
            return

        self.accepted += 1
        if decoded['valid'] or self.publish_invalid:
            values = Float64MultiArray()
            values.data = [float(decoded[signal.name]) for signal in codec.spec.signals]
            self.signal_pub.publish(values)
        self._publish_status(codec, decoded)

    def _publish_status(self, codec, decoded):
        counter = 'ok' if decoded['counter_ok'] else f"jump({decoded['counter_jump']})"
        violations = decoded['range_violations']
        text = (f"0x{codec.frame_id:03X} {codec.name} "
                f"crc={'ok' if decoded['crc_ok'] else 'fail'} "
                f"counter={counter} "
                f"range={'ok' if not violations else '|'.join(violations)} "
                f"accepted={self.accepted} rejected={self.rejected}")
        message = String()
        message.data = text
        self.status_pub.publish(message)
        self.get_logger().info(text, throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = CanDecodeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

