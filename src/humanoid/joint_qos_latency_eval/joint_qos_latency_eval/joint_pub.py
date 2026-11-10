"""按固定频率发布关节指令，用于 QoS / 时延实验。

用法::

    ros2 run joint_qos_latency_eval joint_qos_pub \
      --ros-args -p qos_profile:=best_effort_depth1 -p rate_hz:=200 -p duration:=10
"""

import math
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .protocol import encode
from .qos_profiles import DEFAULT_PROFILE, make

# 六个关节的正弦初相位，避免所有关节同相位导致曲线重合
JOINT_PHASES = (0.0, math.pi / 2.0, math.pi, 0.0, math.pi / 2.0, math.pi)


def _declare_float(node, name, default):
    """声明一个数值参数并返回 float。

    不带默认值声明（NOT_SET），参数保持动态类型：命令行 ``-p rate_hz:=200``
    或 launch 里 ``duration:=10`` 传进来的是 int，若按 double 静态声明会直接
    抛 InvalidParameterTypeException。Foxy 的 ParameterDescriptor 没有
    dynamic_typing 字段（Humble 才有），所以用这种方式兼容两种写法。

    注意：这是为兼容 Foxy 保留的有意写法。在 Humble 及以上运行会打印
    "declaring a parameter only providing its name is deprecated" 警告，
    属预期、不影响功能；若未来只支持 Humble+，可改为
    ``ParameterDescriptor(dynamic_typing=True)``。
    """
    node.declare_parameter(name)
    value = node.get_parameter(name).value
    return default if value is None else float(value)


class JointCommandPublisher(Node):

    def __init__(self):
        super().__init__('joint_qos_pub')
        self.rate_hz = _declare_float(self, 'rate_hz', 200.0)
        self.duration = _declare_float(self, 'duration', 10.0)
        self.amplitude = _declare_float(self, 'amplitude', 0.5)
        start_timeout = _declare_float(self, 'start_timeout', 5.0)
        self.declare_parameter('qos_profile', DEFAULT_PROFILE)
        self.declare_parameter('topic', '/joint_commands')

        profile_name = str(self.get_parameter('qos_profile').value)
        topic = str(self.get_parameter('topic').value)

        if self.rate_hz <= 0.0:
            raise ValueError('rate_hz must be positive, got %r' % self.rate_hz)

        self._publisher = self.create_publisher(
            Float64MultiArray, topic, make(profile_name))
        self._wait_for_subscriber(start_timeout)
        self._timer = self.create_timer(1.0 / self.rate_hz, self._on_timer)

        self._start = time.monotonic()
        self._seq = 0
        self._deadline = (self._start + self.duration
                          if self.duration > 0.0 else float('inf'))

        self.get_logger().info(
            'publish %s at %.1f Hz, qos=%s, duration=%.1f s'
            % (topic, self.rate_hz, profile_name, self.duration))

    @property
    def sent(self):
        return self._seq

    def actual_rate(self):
        """实际发布频率；与 rate_hz 的差即定时器欠触发程度。"""
        elapsed = time.monotonic() - self._start
        return self._seq / elapsed if elapsed > 0.0 else 0.0

    def _wait_for_subscriber(self, timeout):
        # launch 同时起两端时，DDS 发现需要几十毫秒；200 Hz 下这段时间发出的
        # 消息会被订阅端如实计入 missing，使 launch 的丢包率明显高于
        # run_sweep.sh（后者先起订阅端）。这里先等到匹配再启动定时器。
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        deadline = time.monotonic() + timeout
        try:
            while self._publisher.get_subscription_count() == 0:
                if time.monotonic() >= deadline:
                    self.get_logger().warning(
                        'no subscriber matched in %.1f s, publishing anyway'
                        % timeout)
                    return
                executor.spin_once(timeout_sec=0.01)
        finally:
            executor.remove_node(self)

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
    try:
        node = JointCommandPublisher()
    except ValueError as exc:
        print('joint_qos_pub: %s' % exc, file=sys.stderr)
        rclpy.shutdown()
        return 1

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        # 1 ms 轮询：定时器周期 5 ms，超时必须明显小于它，否则会漏触发，
        # 实测用 10 ms 超时会把 200 Hz 拖到 130~180 Hz，各场景频率就不一致了。
        # 不用 rclpy.spin()：在 Foxy 里从回调中调用 rclpy.shutdown() 会死锁。
        # 也不用模块级 spin_once：它每次调用都要对全局 executor 增删节点，
        # 1 ms 轮询下每秒上千次，这部分开销会直接计进被测时延。
        while rclpy.ok() and not node.deadline_reached():
            executor.spin_once(timeout_sec=0.001)
        node.get_logger().info(
            'published %d messages (target %.1f Hz, actual %.1f Hz)'
            % (node.sent, node.rate_hz, node.actual_rate()))
    except KeyboardInterrupt:
        node.get_logger().info('interrupted, published %d messages' % node.sent)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
