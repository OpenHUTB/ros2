import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from collections import deque


class MeanFilterNode(Node):
    def __init__(self):
        super().__init__('mean_filter_node')
        self.declare_parameter('input_topic', '/sensor/raw')
        self.declare_parameter('output_topic', '/sensor/filtered')
        self.declare_parameter('window_size', 5)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        window_size = self.get_parameter('window_size').value

        if window_size < 1:
            raise ValueError(
                f'window_size must be a positive integer, got {window_size}')

        self.window = deque(maxlen=window_size)
        self.sub = self.create_subscription(Float64, input_topic, self.callback, 10)
        self.pub = self.create_publisher(Float64, output_topic, 10)

        self.get_logger().info(f'Mean filter started: {input_topic} -> {output_topic}, window={window_size}')

    def callback(self, msg):
        self.window.append(msg.data)
        avg = sum(self.window) / len(self.window)
        out = Float64()
        out.data = avg
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MeanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
