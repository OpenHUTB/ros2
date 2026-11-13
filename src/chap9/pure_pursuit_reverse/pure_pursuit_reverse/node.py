"""ROS 2 节点：倒车纯追踪控制器。"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from pure_pursuit_reverse import core

PATH = core.DEFAULT_PATH


class ReversePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_reverse")
        self.declare_parameter("speed", 0.25)
        self.declare_parameter("lookahead", 0.9)
        self.declare_parameter("max_steer_rate", 1.2)
        self.declare_parameter("wheelbase", 0.5)
        self.declare_parameter("log_period", 0.5)

        self.speed = float(self.get_parameter("speed").value)
        self.lookahead = float(self.get_parameter("lookahead").value)
        self.max_steer_rate = float(self.get_parameter("max_steer_rate").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.log_period = float(self.get_parameter("log_period").value)

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status = self.create_publisher(String, "/reverse_pursuit/status", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)

        self.steer = 0.0
        self.pose = (0.6, 0.05, math.pi)
        self.last_log = 0.0
        self.max_lateral = 0.0
        self.get_logger().info(
            "reverse pure pursuit: speed=%.2f lookahead=%.2f steer_rate=%.2f"
            % (self.speed, self.lookahead, self.max_steer_rate))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose = (p.x, p.y, math.atan2(siny, cosy))

        s_now, lateral, _ = core.project_to_path(self.pose, PATH)
        if s_now >= core.path_length(PATH) - 0.15:
            self.pub.publish(Twist())
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_log >= self.log_period:
                self.last_log = now
                self.get_logger().info(
                    "goal reached: s=%.2f/%.2f lateral=%+.3f max_lateral=%.3f"
                    % (s_now, core.path_length(PATH), lateral, self.max_lateral))
            return
        target = core.lookahead_point(PATH, s_now, self.lookahead)
        kappa, dist = core.pure_pursuit_curvature(self.pose, target, reverse=True)
        desired = kappa * self.speed
        dt = 0.05
        self.steer = core.rate_limit(self.steer, desired, self.max_steer_rate * dt)

        twist = Twist()
        twist.linear.x = -abs(self.speed)
        twist.angular.z = self.steer
        self.pub.publish(twist)

        self.max_lateral = max(self.max_lateral, abs(lateral))
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log >= self.log_period:
            self.last_log = now
            self.status.publish(String(data="%.4f,%.4f,%.4f" % (kappa, lateral, self.steer)))
            self.get_logger().info(
                "pose=(%.2f,%.2f) kappa=%+.3f lateral=%+.3f steer=%+.3f"
                % (self.pose[0], self.pose[1], kappa, lateral, self.steer))


def main(args=None):
    rclpy.init(args=args)
    node = ReversePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("max_abs_lateral=%.3f m" % node.max_lateral)
        node.destroy_node()
        rclpy.shutdown()
