"""使用单调时间约束 ROS 2 高度目标话题."""
import time

from .core import AltitudeGuard


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64, String

    class GuardNode(Node):
        def __init__(self):
            super().__init__('uav_altitude_command_guard')
            for name, value in [('minimum', 0.4), ('maximum', 3.0), ('rate', 0.5),
                                ('timeout', 0.6), ('initial', 1.5)]:
                self.declare_parameter(name, value)
            self.guard = AltitudeGuard(**{name: self.get_parameter(name).value
                                         for name in ('minimum', 'maximum', 'rate', 'timeout', 'initial')})
            self.output = self.create_publisher(Float64, '/uav/altitude_target_safe', 10)
            self.status = self.create_publisher(String, '/uav/altitude_guard_status', 10)
            self.subscription = self.create_subscription(Float64, '/uav/altitude_target', self.receive, 10)
            self.timer = self.create_timer(0.02, self.tick)

        def receive(self, message):
            if not self.guard.command(message.data, time.monotonic()):
                self.get_logger().warning('已拒绝非有限高度指令')

        def tick(self):
            value, state = self.guard.tick(time.monotonic())
            self.output.publish(Float64(data=value))
            self.status.publish(String(data=state))

    rclpy.init(args=args)
    node = None
    try:
        node = GuardNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
