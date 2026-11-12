"""离线演示用的激光雷达仿真节点。

按 /cmd_vel 积分差速小车位姿，对世界模型做射线投射，发布带强度的 /scan。
世界里的玻璃板与黑色泡棉用低反射率标记，用来复现"几何上是障碍、回波却很弱"
的场景。原始轨迹写入 CSV，供 make_figure.py 画图。
"""

import csv
import math
import random

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from lidar_intensity_traversability import core

# 世界模型：线段 (x1, y1, x2, y2) + 反射率
SEGMENTS = [
    (0.0, 1.20, 12.0, 1.20, 1.00),   # 左墙
    (0.0, -1.20, 12.0, -1.20, 1.00),  # 右墙
    (4.00, 0.05, 4.00, 1.20, 0.12),   # 玻璃板（挡住左半边）
    (7.00, -1.20, 7.00, -0.05, 0.08),  # 黑色泡棉（挡住右半边）
    (12.0, -1.20, 12.0, 1.20, 1.00),  # 尽头墙
]


def ray_segment(px, py, dx, dy, seg):
    """射线与线段求交，返回距离或 None。"""
    x1, y1, x2, y2, _ = seg
    sx, sy = x2 - x1, y2 - y1
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-9:
        return None
    t = ((x1 - px) * sy - (y1 - py) * sx) / denom
    u = ((x1 - px) * dy - (y1 - py) * dx) / denom
    if t > 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


class ScanSim(Node):
    def __init__(self):
        super().__init__("scan_sim")
        self.declare_parameter("csv_path", "/tmp/lidar_intensity_demo.csv")
        self.declare_parameter("rate_hz", 15.0)
        self.csv_path = self.get_parameter("csv_path").value
        self.rate = float(self.get_parameter("rate_hz").value)

        self.x = 0.3
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_min_eff = 0.0
        self.last_action = ""

        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(String, "/traversability/decision", self.on_decision, 10)
        self.done = False
        self.create_timer(1.0 / self.rate, self.step)

        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(["t", "x", "y", "theta", "v", "w", "min_eff", "action"])
        self.t = 0.0
        self.get_logger().info("scan_sim world with glass(4.0) and foam(7.0), csv=" + self.csv_path)

    def on_cmd(self, msg):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def on_decision(self, msg):
        parts = msg.data.split(",")
        if len(parts) == 3:
            self.last_min_eff = float(parts[0])
            self.last_action = parts[2]

    def step(self):
        dt = 1.0 / self.rate
        self.theta += self.w * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.t += dt

        n = 271
        angle_min = -math.pi * 0.75
        angle_inc = math.pi * 1.5 / (n - 1)
        ranges = []
        intensities = []
        for i in range(n):
            a = self.theta + angle_min + i * angle_inc
            dx, dy = math.cos(a), math.sin(a)
            best_t = None
            best_refl = 1.0
            for seg in SEGMENTS:
                t = ray_segment(self.x, self.y, dx, dy, seg)
                if t is not None and (best_t is None or t < best_t):
                    best_t = t
                    best_refl = seg[4]
            if best_t is None or best_t > 12.0:
                ranges.append(float("inf"))
                intensities.append(0.0)
            else:
                ranges.append(best_t)
                base = 1400.0 * best_refl
                intensities.append(max(0.0, base * math.exp(-0.12 * best_t)
                                       + random.uniform(-5.0, 5.0)))

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser"
        scan.angle_min = angle_min
        scan.angle_max = angle_min + angle_inc * (n - 1)
        scan.angle_increment = angle_inc
        scan.range_min = 0.05
        scan.range_max = 12.0
        scan.ranges = ranges
        scan.intensities = intensities
        self.scan_pub.publish(scan)

        odom = Odometry()
        odom.header.stamp = scan.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        self.odom_pub.publish(odom)

        self.csv.writerow([round(self.t, 3), round(self.x, 3), round(self.y, 3),
                           round(self.theta, 3), round(self.v, 3), round(self.w, 3),
                           round(self.last_min_eff, 3), self.last_action])
        if int(self.t * 10) % 50 == 0:
            self.csv_file.flush()
        if self.t > 45.0 and not self.done:
            self.csv_file.close()
            self.get_logger().info("sim finished at t=%.1fs" % self.t)
            self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = ScanSim()
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
