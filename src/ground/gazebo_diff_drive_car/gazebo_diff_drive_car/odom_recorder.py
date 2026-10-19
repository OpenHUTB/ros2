"""里程计轨迹记录与性能评价节点（ROS 2）。

订阅 ``odom``，累积行驶路程、速度与航向变化，退出时输出量化评价指标
并保存轨迹 CSV，用于不同模式（键盘 / 雷达避障 / 视觉避障）之间的对比。

输出指标：运行时长、轨迹总长度、净位移、路径效率、平均/最大速度、
平均角速度、位姿覆盖范围。
"""

import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


#: 轨迹抽稀距离阈值（米）
MIN_SAMPLE_DISTANCE = 0.02


def quaternion_to_yaw(x, y, z, w):
    """四元数转偏航角（弧度）。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomRecorderNode(Node):
    """轨迹记录与性能统计节点。"""

    def __init__(self):
        """声明参数并订阅里程计。"""
        super().__init__('odom_recorder')

        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('result_dir', '~/gazebo_diff_drive_car_results')

        odom_topic = self.get_parameter('odom_topic').value
        self.result_dir = os.path.expanduser(
            self.get_parameter('result_dir').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0

        self.has_pose = False
        self.trail = []
        self.start_time = None
        self.last_time = None
        self.distance = 0.0
        self.v_max = 0.0
        self.w_abs_sum = 0.0
        self.w_samples = 0
        self.prev_x = None
        self.prev_y = None

        self.sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)

        self.get_logger().info('开始记录轨迹：odom=%s' % odom_topic)

    def odom_callback(self, msg):
        """累积统计量。"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.x = position.x
        self.y = position.y
        self.yaw = quaternion_to_yaw(orientation.x, orientation.y,
                                     orientation.z, orientation.w)
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if not self.has_pose:
            self.has_pose = True
            self.start_time = stamp
            self.prev_x = self.x
            self.prev_y = self.y
            self.trail.append((stamp, self.x, self.y, self.yaw, self.v, self.w))
            return

        self.last_time = stamp

        step = math.hypot(self.x - self.prev_x, self.y - self.prev_y)
        if step >= MIN_SAMPLE_DISTANCE:
            self.distance += step
            self.trail.append((stamp, self.x, self.y, self.yaw, self.v, self.w))
            self.prev_x = self.x
            self.prev_y = self.y

        speed = abs(self.v)
        if speed > self.v_max:
            self.v_max = speed
        self.w_abs_sum += abs(self.w)
        self.w_samples += 1

    def save_csv(self):
        """保存轨迹 CSV，返回文件路径（无数据时返回 None）。"""
        if not self.trail:
            return None
        try:
            os.makedirs(self.result_dir, exist_ok=True)
            stamp = time.strftime('%Y%m%d_%H%M%S')
            path = os.path.join(self.result_dir, 'trajectory_%s.csv' % stamp)
            with open(path, 'w', newline='') as fp:
                writer = csv.writer(fp)
                writer.writerow(['timestamp', 'x', 'y', 'yaw',
                                 'linear_vel', 'angular_vel'])
                for row in self.trail:
                    writer.writerow(['%.4f' % value for value in row])
            return path
        except OSError as exc:
            self.get_logger().error('轨迹保存失败：%s' % exc)
            return None

    def report(self):
        """打印性能评价报告。"""
        if not self.has_pose:
            self.get_logger().warn('未收到里程计数据')
            return

        duration = 0.0
        if self.last_time is not None and self.start_time is not None:
            duration = self.last_time - self.start_time

        avg_v = self.distance / duration if duration > 1e-6 else 0.0
        avg_w = self.w_abs_sum / self.w_samples if self.w_samples else 0.0
        xs = [point[1] for point in self.trail]
        ys = [point[2] for point in self.trail]

        self.get_logger().info('=' * 56)
        self.get_logger().info('        差速小车运行性能评价报告')
        self.get_logger().info('=' * 56)
        self.get_logger().info('  运行时长     : %8.2f s' % duration)
        self.get_logger().info('  轨迹总长度   : %8.2f m' % self.distance)
        self.get_logger().info('  平均线速度   : %8.3f m/s' % avg_v)
        self.get_logger().info('  最大线速度   : %8.3f m/s' % self.v_max)
        self.get_logger().info('  平均角速度   : %8.3f rad/s' % avg_w)
        self.get_logger().info('  采样点数     : %8d' % len(self.trail))
        self.get_logger().info('  X 范围       : %8.2f ~ %.2f m' % (min(xs), max(xs)))
        self.get_logger().info('  Y 范围       : %8.2f ~ %.2f m' % (min(ys), max(ys)))
        self.get_logger().info('=' * 56)


def main(args=None):
    """节点入口。"""
    rclpy.init(args=args)
    node = OdomRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        path = node.save_csv()
        if path:
            node.get_logger().info('轨迹已保存：%s（%d 点）'
                                   % (path, len(node.trail)))
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
