"""ROS 2 节点：LOS 视线制导航点跟踪。

话题：
    订阅  /uav/odom         nav_msgs/Odometry
    发布  /uav/cmd_vel      geometry_msgs/Twist（机体系速度）
    发布  /uav/los_state    std_msgs/String（诊断）

参数由 launch 以标准 ROS 2 方式加载（config/params.yaml，节点名下的
ros__parameters），因此命令行 -p/--ros-args 覆盖同样生效。
控制器由独立 timer 驱动，odom 只负责更新状态。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from uav_los_waypoint_guidance import core

DEFAULT_WAYPOINTS = [4.0, 0.0, 8.0, 3.0, 12.0, 0.0, 16.0, -3.0]


class LosGuidanceNode(Node):
    def __init__(self):
        super().__init__("uav_los_waypoint_guidance")
        self.declare_parameter("speed", 1.5)
        self.declare_parameter("lookahead", 2.0)
        self.declare_parameter("accept_radius", 0.8)
        self.declare_parameter("log_period", 0.5)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("yaw_gain", 1.2)
        self.declare_parameter("max_yaw_rate", 0.8)
        self.declare_parameter("waypoints", DEFAULT_WAYPOINTS)

        self.speed = float(self.get_parameter("speed").value)
        self.lookahead = float(self.get_parameter("lookahead").value)
        self.accept_radius = float(self.get_parameter("accept_radius").value)
        self.log_period = float(self.get_parameter("log_period").value)
        self.control_rate = float(self.get_parameter("control_rate_hz").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)

        raw = [float(v) for v in self.get_parameter("waypoints").value]
        if len(raw) < 4 or len(raw) % 2 != 0:
            raise ValueError("waypoints 需要成对的 x,y，至少两个航点")
        self.wps = [(raw[i], raw[i + 1]) for i in range(0, len(raw), 2)]

        self.index = 0
        self.prev = (0.0, 0.0)
        self.pose = None
        self.done = False
        self.last_log = 0.0

        self.cmd_pub = self.create_publisher(Twist, "/uav/cmd_vel", 10)
        self.state_pub = self.create_publisher(String, "/uav/los_state", 10)
        self.create_subscription(Odometry, "/uav/odom", self.on_odom, 10)
        if self.control_rate <= 0.0:
            raise ValueError("control_rate_hz 必须为正")
        self.create_timer(1.0 / self.control_rate, self.control_step)
        self.get_logger().info(
            "LOS guidance: %d waypoints speed=%.2f lookahead=%.2f accept=%.2f rate=%.0fHz"
            % (len(self.wps), self.speed, self.lookahead, self.accept_radius, self.control_rate))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose = (p.x, p.y, math.atan2(siny, cosy))

    def control_step(self):
        if self.done:
            self.cmd_pub.publish(Twist())
            return
        if self.pose is None:
            return

        wp = self.wps[self.index]
        if core.waypoint_reached(self.pose, wp, self.accept_radius):
            self.get_logger().info("waypoint %d reached at (%.2f, %.2f)"
                                   % (self.index, wp[0], wp[1]))
            self.prev = wp
            self.index += 1
            if self.index >= len(self.wps):
                self.done = True
                self.cmd_pub.publish(Twist())
                self.get_logger().info("all waypoints reached")
                return
            wp = self.wps[self.index]

        vx, vy, err, dist = core.guidance_step(
            self.pose, self.pose[2], self.prev, wp, self.speed, self.lookahead)

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = max(-self.max_yaw_rate,
                              min(self.max_yaw_rate, self.yaw_gain * err))
        self.cmd_pub.publish(twist)

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log >= self.log_period:
            self.last_log = now
            self.state_pub.publish(String(data="%d,%.3f,%.3f,%.3f" % (self.index, dist, err, vx)))
            self.get_logger().info(
                "wp=%d dist=%.2f heading_err=%+6.1f deg vx=%.2f vy=%.2f"
                % (self.index, dist, math.degrees(err), vx, vy))


def main(args=None):
    rclpy.init(args=args)
    node = LosGuidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
