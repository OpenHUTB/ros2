"""使用单调时间约束 ROS 2 高度目标话题."""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

from .core import AltitudeGuard

# 参数名与默认值，启动时读取一次。话题名保持相对，便于 launch 里加 namespace。
PARAMETERS = (('minimum', 0.4), ('maximum', 3.0), ('rate', 0.5),
              ('timeout', 0.6), ('initial', 1.5))


class AltitudeGuardNode(Node):
    """订阅原始高度目标，发布限幅、限速后的安全目标."""

    def __init__(self):
        super().__init__('uav_altitude_command_guard')
        for name, value in PARAMETERS:
            self.declare_parameter(name, value)
        self.guard = AltitudeGuard(**{name: self.get_parameter(name).value
                                     for name, _ in PARAMETERS})
        self.output = self.create_publisher(Float64, 'uav/altitude_target_safe', 10)
        self.status = self.create_publisher(String, 'uav/altitude_guard_status', 10)
        self.subscription = self.create_subscription(Float64, 'uav/altitude_target', self.receive, 10)
        self.timer = self.create_timer(0.02, self.tick)

    def receive(self, message):
        """接收新目标；非有限输入被拒绝，且不刷新有效指令时间."""
        if not self.guard.command(message.data, time.monotonic()):
            self.get_logger().warning('已拒绝非有限高度指令')

    def tick(self):
        """按 50 Hz 推进，并发布安全目标与当前状态."""
        value, state = self.guard.tick(time.monotonic())
        self.output.publish(Float64(data=value))
        self.status.publish(String(data=state))


def main(args=None):
    """启动节点并阻塞在 spin 中."""
    rclpy.init(args=args)
    node = None
    try:
        node = AltitudeGuardNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
