"""nav2 超时兜底：收到 /goal_pose 且 nav2 停发时，用前向扇区避障自己走。"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from .nav_logic import sector_speed


class SimpleNavNode(Node):
    def __init__(self):
        super().__init__("ugv_simple_nav")
        self._scan = None
        self._has_goal = False
        self._pub = self.create_publisher(Twist, "/simple_nav/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._goal_cb, 10)
        self.create_timer(0.1, self._tick)

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    def _goal_cb(self, msg: PoseStamped):
        self._has_goal = True

    def _tick(self):
        if not self._has_goal or self._scan is None:
            return
        vx = sector_speed(list(self._scan.ranges),
                          self._scan.angle_min,
                          self._scan.angle_increment)
        t = Twist()
        t.linear.x = vx
        self._pub.publish(t)


def main():
    rclpy.init()
    n = SimpleNavNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
