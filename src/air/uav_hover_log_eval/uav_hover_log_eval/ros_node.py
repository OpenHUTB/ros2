"""定期发布指定定高日志的指标，不控制飞行器。"""
import json

from .metrics import load_log, summarize
from .paths import resource


def main(args=None):
    """按参数计算一次指标，并以 1 Hz 发布 JSON 摘要."""
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String

    class MetricsNode(Node):
        def __init__(self):
            super().__init__('uav_hover_log_eval')
            self.declare_parameter('log_path', str(resource('data/gust_ppo_101.csv')))
            self.declare_parameter('disturbance_end', 3.0)
            self.declare_parameter('disturbance_window', 1.0)
            self.declare_parameter('tolerance', 0.15)
            self.declare_parameter('hold_s', 0.5)
            self.result = summarize(
                load_log(self.get_parameter('log_path').value),
                disturbance_end=float(self.get_parameter('disturbance_end').value),
                tolerance=float(self.get_parameter('tolerance').value),
                hold_s=float(self.get_parameter('hold_s').value),
                disturbance_window=float(
                    self.get_parameter('disturbance_window').value))
            self.publisher = self.create_publisher(String, '/uav/hover_metrics', 10)
            self.timer = self.create_timer(1.0, self.publish)

        def publish(self):
            self.publisher.publish(String(data=json.dumps(self.result)))

    rclpy.init(args=args)
    node = None
    try:
        node = MetricsNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
