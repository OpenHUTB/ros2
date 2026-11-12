"""ROS 2 滑动窗口均值滤波节点."""

from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class MovingAverageFilter:
    """
    固定长度的滑动窗口均值滤波器.

    窗口未填满时按已有样本计算均值，避免启动阶段把未采集的数据
    当作 0 参与平均。
    """

    def __init__(self, window_size):
        """
        创建滤波器并校验窗口长度.

        :param window_size: 窗口长度，必须是正整数.
        :raises ValueError: 窗口长度不是正整数时抛出.
        """
        if isinstance(window_size, bool) or not isinstance(window_size, int):
            raise ValueError(
                f'window_size must be an integer, got {window_size!r}')
        if window_size < 1:
            raise ValueError(
                f'window_size must be >= 1, got {window_size}')
        self._window = deque(maxlen=window_size)

    @property
    def window_size(self):
        """返回窗口长度."""
        return self._window.maxlen

    def update(self, value):
        """加入一个样本并返回当前窗口内的均值."""
        self._window.append(float(value))
        return sum(self._window) / len(self._window)

    def reset(self):
        """清空窗口，回到启动时的状态."""
        self._window.clear()


class MeanFilterNode(Node):
    """订阅浮点话题并发布滑动窗口均值滤波结果."""

    def __init__(self):
        """读取参数、创建滤波器并建立发布订阅关系."""
        super().__init__('mean_filter_node')
        self.declare_parameter('input_topic', '/sensor/raw')
        self.declare_parameter('output_topic', '/sensor/filtered')
        self.declare_parameter('window_size', 5)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        window_size = self.get_parameter('window_size').value

        try:
            self.filter = MovingAverageFilter(window_size)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            raise

        self.subscription = self.create_subscription(
            Float64, self.input_topic, self.callback, 10)
        self.publisher = self.create_publisher(
            Float64, self.output_topic, 10)

        self.get_logger().info(
            f'Mean filter started: {self.input_topic} -> '
            f'{self.output_topic}, window={window_size}')

    def callback(self, msg):
        """对每条输入样本执行滑动平均并发布."""
        out = Float64()
        out.data = self.filter.update(msg.data)
        self.publisher.publish(out)


def main(args=None):
    """启动节点并保持运行，直到收到 Ctrl+C."""
    rclpy.init(args=args)
    node = None
    try:
        node = MeanFilterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
