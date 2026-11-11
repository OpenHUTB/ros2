"""订阅 IMU 数据，用指定方法解算姿态并记录估计值。

方法：
  complementary  互补滤波，alpha 为陀螺仪权重
  madgwick       Madgwick IMU 版，beta 为梯度下降增益
"""

import csv
import math
import os

import rclpy
from geometry_msgs.msg import QuaternionStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu

from humanoid_imu_attitude.quat_math import (
    accel_tilt_quat,
    complementary_step,
    euler_from_quat,
    madgwick_step,
)

METHODS = ('complementary', 'madgwick')


class AttitudeFilter(Node):

    def __init__(self):
        super().__init__('attitude_filter')
        self.declare_parameter('method', 'complementary')
        self.declare_parameter('profile', 'unknown')
        self.declare_parameter('alpha', 0.98)
        self.declare_parameter('beta', 0.05)
        self.declare_parameter('duration', 20.0)
        self.declare_parameter('out_dir', 'results')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('attitude_topic', '')

        self.method = self.get_parameter('method').value
        if self.method not in METHODS:
            raise ValueError('method 必须是 %s 之一' % (METHODS,))
        self.profile = self.get_parameter('profile').value
        self.alpha = float(self.get_parameter('alpha').value)
        self.beta = float(self.get_parameter('beta').value)
        self.duration = float(self.get_parameter('duration').value)
        self.out_dir = self.get_parameter('out_dir').value

        attitude_topic = self.get_parameter('attitude_topic').value
        if not attitude_topic:
            attitude_topic = '/imu/attitude/%s' % self.method
        self.pub = self.create_publisher(QuaternionStamped, attitude_topic, 10)
        self.sub = self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self.on_imu, 20)

        self.q = None
        self.last_stamp = None
        self.rows = []
        self.first_stamp = None
        self.finished = False
        self.saved = False
        self.timer = self.create_timer(0.5, self.check_finished)
        self.get_logger().info('attitude_filter: method=%s topic=%s' % (self.method, attitude_topic))

    def on_imu(self, msg):
        if self.finished:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        accel = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        gyro = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        )

        if self.q is None:
            # 第一帧只用加速度计确定初始 roll/pitch，避免从单位四元数慢慢收敛
            self.q = accel_tilt_quat(accel)
            self.first_stamp = stamp
            self.last_stamp = stamp
            return

        dt = stamp - self.last_stamp
        self.last_stamp = stamp
        if dt <= 1e-4:
            return
        dt = min(dt, 0.05)

        if self.method == 'complementary':
            self.q = complementary_step(self.q, gyro, accel, dt, self.alpha)
        else:
            self.q = madgwick_step(self.q, gyro, accel, dt, self.beta)

        out = QuaternionStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'imu_link'
        out.quaternion.w = float(self.q[0])
        out.quaternion.x = float(self.q[1])
        out.quaternion.y = float(self.q[2])
        out.quaternion.z = float(self.q[3])
        self.pub.publish(out)

        roll, pitch, yaw = euler_from_quat(self.q)
        self.rows.append((stamp, math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))

    def check_finished(self):
        if self.finished:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.first_stamp is None:
            return
        if now - self.first_stamp < self.duration + 0.5:
            return
        self.finished = True

    def save(self):
        if self.saved:
            return
        self.saved = True
        os.makedirs(self.out_dir, exist_ok=True)
        path = os.path.join(self.out_dir, '%s_%s.csv' % (self.profile, self.method))
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'roll_deg', 'pitch_deg', 'yaw_deg'])
            writer.writerows(self.rows)  # t 列是绝对 ROS 时钟秒数  # t 列是绝对 ROS 时钟秒数
        self.get_logger().info('已保存 %d 条估计到 %s' % (len(self.rows), path))


def main(args=None):
    rclpy.init(args=args)
    node = AttitudeFilter()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
