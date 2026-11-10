"""唯一 cmd_vel 发布者：仲裁 nav2 / simple_nav，限幅。"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .nav_logic import relay, Limits


class NavRelayNode(Node):
    def __init__(self):
        super().__init__("ugv_nav_relay")
        self._nav2 = Twist()
        self._simple = Twist()
        self._last_nav2 = 0.0
        self._timeout = self.declare_parameter("nav2_timeout", 1.5).value
        self._limits = Limits(
            max_vx=self.declare_parameter("max_vx", 0.5).value,
            max_yaw=self.declare_parameter("max_yaw", 1.0).value,
        )
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Twist, "/nav2/cmd_vel", self._nav2_cb, 10)
        self.create_subscription(Twist, "/simple_nav/cmd_vel", self._simple_cb, 10)
        self.create_timer(0.1, self._tick)

    def _nav2_cb(self, msg: Twist):
        self._nav2 = msg
        self._last_nav2 = time.time()

    def _simple_cb(self, msg: Twist):
        self._simple = msg

    def _tick(self):
        alive = (time.time() - self._last_nav2) < self._timeout
        vx, yaw = relay(self._nav2.linear.x, self._nav2.angular.z,
                        self._simple.linear.x, self._simple.angular.z,
                        alive, self._limits)
        t = Twist()
        t.linear.x = vx
        t.angular.z = yaw
        self._pub.publish(t)


def main():
    rclpy.init()
    n = NavRelayNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
