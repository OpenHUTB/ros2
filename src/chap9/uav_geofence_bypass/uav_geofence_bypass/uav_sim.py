"""无人机质点仿真：跟随 /uav/safe_goal，发布 /uav/odom。"""

import csv
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class UavSim(Node):
    def __init__(self):
        super().__init__("uav_sim")
        self.declare_parameter("csv_path", "/tmp/uav_geofence_bypass.csv")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("duration", 60.0)
        self.declare_parameter("speed", 1.5)
        self.csv_path = self.get_parameter("csv_path").value
        self.rate = float(self.get_parameter("rate_hz").value)
        self.duration = float(self.get_parameter("duration").value)
        self.speed = float(self.get_parameter("speed").value)

        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.gx, self.gy = 15.0, 0.0
        self.t = 0.0
        self.done = False

        self.odom_pub = self.create_publisher(Odometry, "/uav/odom", 10)
        self.create_subscription(PoseStamped, "/uav/safe_goal", self.on_goal, 10)
        self.create_timer(1.0 / self.rate, self.step)
        self.f = open(self.csv_path, "w", newline="")
        self.csv = csv.writer(self.f)
        self.csv.writerow(["t", "x", "y", "gx", "gy"])
        self.get_logger().info("uav_sim start (0,0) csv=" + self.csv_path)

    def on_goal(self, msg):
        self.gx = msg.pose.position.x
        self.gy = msg.pose.position.y

    def step(self):
        dt = 1.0 / self.rate
        dx, dy = self.gx - self.x, self.gy - self.y
        dist = math.hypot(dx, dy)
        if dist > 0.15:
            desired = math.atan2(dy, dx)
            self.yaw = desired
            step = min(self.speed * dt, dist)
            self.x += step * math.cos(desired)
            self.y += step * math.sin(desired)
        self.t += dt

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.odom_pub.publish(odom)

        self.csv.writerow([round(self.t, 3), round(self.x, 4), round(self.y, 4),
                           round(self.gx, 4), round(self.gy, 4)])
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
