"""无人机质点运动学仿真：订阅 /uav/cmd_vel（机体系速度），发布 /uav/odom。"""

import csv
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class UavSim(Node):
    def __init__(self):
        super().__init__("uav_sim")
        self.declare_parameter("csv_path", "/tmp/uav_los_waypoint_guidance.csv")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("duration", 60.0)
        self.csv_path = self.get_parameter("csv_path").value
        self.rate = float(self.get_parameter("rate_hz").value)
        self.duration = float(self.get_parameter("duration").value)

        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.vx, self.vy, self.wz = 0.0, 0.0, 0.0
        self.t = 0.0
        self.done = False

        self.odom_pub = self.create_publisher(Odometry, "/uav/odom", 10)
        self.create_subscription(Twist, "/uav/cmd_vel", self.on_cmd, 10)
        self.create_timer(1.0 / self.rate, self.step)
        self.f = open(self.csv_path, "w", newline="")
        self.csv = csv.writer(self.f)
        self.csv.writerow(["t", "x", "y", "yaw", "vx", "vy"])
        self.get_logger().info("uav_sim start at origin, csv=" + self.csv_path)

    def on_cmd(self, msg):
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.wz = float(msg.angular.z)

    def step(self):
        dt = 1.0 / self.rate
        self.yaw += self.wz * dt
        wx = self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)
        wy = self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)
        self.x += wx * dt
        self.y += wy * dt
        self.t += dt

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.odom_pub.publish(odom)

        self.csv.writerow([round(self.t, 3), round(self.x, 4), round(self.y, 4),
                           round(self.yaw, 4), round(self.vx, 4), round(self.vy, 4)])
        if self.t >= self.duration:
            self.f.close()
            self.done = True
            self.get_logger().info("sim finished at (%.2f, %.2f)" % (self.x, self.y))


def main(args=None):
    rclpy.init(args=args)
    node = UavSim()
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
