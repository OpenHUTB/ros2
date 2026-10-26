"""定期发布指定定高日志的指标，不控制飞行器。"""
import json
from .metrics import load_log, summarize
from .paths import resource


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class MetricsNode(Node):
        def __init__(self):
            super().__init__('uav_hover_log_eval')
            self.declare_parameter('log_path', str(resource('data/gust_ppo_101.csv')))
            self.result = summarize(load_log(self.get_parameter('log_path').value))
            self.publisher = self.create_publisher(String, '/uav/hover_metrics', 10)
            self.timer = self.create_timer(1.0, self.publish)

        def publish(self):
            self.publisher.publish(String(data=json.dumps(self.result)))

    rclpy.init(args=args)
    node = None
    try:
        node = MetricsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
