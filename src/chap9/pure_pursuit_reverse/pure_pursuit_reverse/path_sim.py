"""倒车工况的差速小车运动学仿真，订阅 /cmd_vel、发布 /odom。"""

import csv
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class PathSim(Node):
    def __init__(self):
        super().__init__("path_sim")
        self.declare_parameter("csv_path", "/tmp/pure_pursuit_reverse.csv")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("duration", 60.0)
        self.csv_path = self.get_parameter("csv_path").value
        self.rate = float(self.get_parameter("rate_hz").value)
        self.duration = float(self.get_parameter("duration").value)

        self.x, self.y, self.theta = 0.6, 0.05, math.pi
        self.v, self.w = 0.0, 0.0
        self.t = 0.0
        self.done = False

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_timer(1.0 / self.rate, self.step)
        self.f = open(self.csv_path, "w", newline="")
        self.csv = csv.writer(self.f)
        self.csv.writerow(["t", "x", "y", "theta", "v", "w"])
        self.get_logger().info("path_sim start pose=(0.60, 0.05, pi) csv=" + self.csv_path)

    def on_cmd(self, msg):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def step(self):
        dt = 1.0 / self.rate
        self.theta += self.w * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.t += dt

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        self.odom_pub.publish(odom)

        self.csv.writerow([round(self.t, 3), round(self.x, 4), round(self.y, 4),
                           round(self.theta, 4), round(self.v, 4), round(self.w, 4)])
        if self.t >= self.duration:
            self.f.close()
            self.done = True
            self.get_logger().info("sim finished: x=%.2f y=%.2f" % (self.x, self.y))


def main(args=None):
    rclpy.init(args=args)
    node = PathSim()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
