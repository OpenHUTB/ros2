"""
把 /auv/odom 记录成 nav_msgs/Path 并可选写入 CSV，用于轨迹可视化.

订阅 /auv/odom，发布 /auv/trajectory 供 RViz 的 Path 显示；
若设置了 csv_path 参数，退出时把采样点写成 CSV，便于离线画图。
"""

import csv
import os
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        self.declare_parameter('csv_path', '')
        self.declare_parameter('max_points', 20000)
        self.csv_path = self.get_parameter('csv_path').value
        self.max_points = self.get_parameter('max_points').value

        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.rows = []

        self.path_pub = self.create_publisher(Path, '/auv/trajectory', 10)
        self.create_subscription(Odometry, '/auv/odom', self.odom_cb, 10)
        self.get_logger().info('轨迹记录节点启动，订阅 /auv/odom')

    def odom_cb(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(pose)
        # 长航程时丢弃最旧的点，避免 Path 无限增长
        if len(self.path.poses) > self.max_points:
            del self.path.poses[0:len(self.path.poses) - self.max_points]
        self.path_pub.publish(self.path)

        position = msg.pose.pose.position
        stamp = msg.header.stamp
        self.rows.append((
            stamp.sec + stamp.nanosec * 1e-9,
            position.x,
            position.y,
            position.z,
        ))

    def write_csv(self):
        if not self.csv_path or not self.rows:
            return
        directory = os.path.dirname(os.path.abspath(self.csv_path))
        os.makedirs(directory, exist_ok=True)
        with open(self.csv_path, 'w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['t', 'x', 'y', 'z'])
            writer.writerows(self.rows)
        self.get_logger().info(
            f'轨迹已写入 {self.csv_path}，共 {len(self.rows)} 个采样点')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()

    def handle_sigint(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_sigint)
    try:
        # 用带超时的 spin_once 循环，保证收到 SIGINT 后能立刻返回，
        # 否则 rclpy.spin 会一直阻塞在 C 层等待，launch 会升级成 SIGKILL
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.write_csv()
        except Exception as exc:
            node.get_logger().error(f'写入轨迹失败: {exc}')
        node.destroy_node()
        # 外部（如 launch）可能已经关闭 context，重复 shutdown 会抛异常
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
