"""按固定频率发布关节指令，用于 QoS / 时延实验。

用法::

    ros2 run joint_qos_latency_eval joint_qos_pub \
      --ros-args -p qos_profile:=best_effort_depth1 -p rate_hz:=200 -p duration:=10
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .protocol import encode
from .qos_profiles import DEFAULT_PROFILE, make

# 六个关节的正弦初相位，避免所有关节同相位导致曲线重合
JOINT_PHASES = (0.0, math.pi / 2.0, math.pi, 0.0, math.pi / 2.0, math.pi)


class JointCommandPublisher(Node):

    def __init__(self):
        super().__init__('joint_qos_pub')
        self.declare_parameter('rate_hz', 200.0)
        self.declare_parameter('qos_profile', DEFAULT_PROFILE)
        self.declare_parameter('duration', 10.0)
        self.declare_parameter('topic', '/joint_commands')
        self.declare_parameter('amplitude', 0.5)

        rate_hz = float(self.get_parameter('rate_hz').value)
        profile_name = str(self.get_parameter('qos_profile').value)
        self.duration = float(self.get_parameter('duration').value)
        topic = str(self.get_parameter('topic').value)
        self.amplitude = float(self.get_parameter('amplitude').value)

        if rate_hz <= 0.0:
            raise ValueError('rate_hz must be positive, got %r' % rate_hz)

        self._publisher = self.create_publisher(
            Float64MultiArray, topic, make(profile_name))
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)

        self._start = time.monotonic()
        self._seq = 0
        self._deadline = (self._start + self.duration
                          if self.duration > 0.0 else float('inf'))

        self.get_logger().info(
            'publish %s at %.1f Hz, qos=%s, duration=%.1f s'
            % (topic, rate_hz, profile_name, self.duration))

    @property
    def sent(self):
        return self._seq

    def _on_timer(self):
        # 取 ROS 时钟纳秒值作为发送时刻，接收端用同一时钟反算单程时延
        stamp_ns = self.get_clock().now().nanoseconds
        phase = 2.0 * math.pi * (time.monotonic() - self._start)
        positions = [self.amplitude * math.sin(phase + offset)
                     for offset in JOINT_PHASES]

        self._publisher.publish(
            Float64MultiArray(data=encode(self._seq, stamp_ns, positions)))
        self._seq += 1

    def deadline_reached(self):
        return time.monotonic() >= self._deadline


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()
    try:
        # 1 ms 轮询：定时器周期 5 ms，超时必须明显小于它，否则会漏触发，
        # 实测用 10 ms 超时会把 200 Hz 拖到 130~180 Hz，各场景频率就不一致了。
        # 不用 rclpy.spin()：在 Foxy 里从回调中调用 rclpy.shutdown() 会死锁。
        while rclpy.ok() and not node.deadline_reached():
            rclpy.spin_once(node, timeout_sec=0.001)
        node.get_logger().info('published %d messages' % node.sent)
    except KeyboardInterrupt:
        node.get_logger().info('interrupted, published %d messages' % node.sent)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
