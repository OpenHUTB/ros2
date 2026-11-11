"""按预设轨迹合成带零偏和噪声的 IMU 数据，并输出真值 CSV。

发布：
  /imu/data          sensor_msgs/Imu         角速度与比方（重力）读数
  /imu/ground_truth  geometry_msgs/QuaternionStamped  真实姿态
"""

import csv
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import QuaternionStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu

from humanoid_imu_attitude.quat_math import gravity_in_body, quat_from_euler, quat_normalize
from humanoid_imu_attitude.trajectory import attitude_at, body_rates_at


def quat_to_msg(q):
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


class ImuSim(Node):

    def __init__(self):
        super().__init__('imu_sim')
        self.declare_parameter('profile', 'slow')
        self.declare_parameter('rate_hz', 200.0)
        self.declare_parameter('duration', 20.0)
        self.declare_parameter('out_dir', 'results')
        self.declare_parameter('gyro_bias_x', 0.004)
        self.declare_parameter('gyro_bias_y', -0.003)
        self.declare_parameter('gyro_bias_z', 0.005)
        self.declare_parameter('gyro_noise', 0.002)
        self.declare_parameter('accel_noise', 0.3)
        self.declare_parameter('seed', 7)

        self.profile = self.get_parameter('profile').value
        self.rate = float(self.get_parameter('rate_hz').value)
        self.duration = float(self.get_parameter('duration').value)
        self.out_dir = self.get_parameter('out_dir').value
        self.bias = np.array([
            float(self.get_parameter('gyro_bias_x').value),
            float(self.get_parameter('gyro_bias_y').value),
            float(self.get_parameter('gyro_bias_z').value),
        ])
        self.gyro_noise = float(self.get_parameter('gyro_noise').value)
        self.accel_noise = float(self.get_parameter('accel_noise').value)
        self.rng = np.random.default_rng(int(self.get_parameter('seed').value))

        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.pub_gt = self.create_publisher(QuaternionStamped, '/imu/ground_truth', 10)
        self.rows = []
        self.finished = False
        self.saved = False
        self._start_sec = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(1.0 / self.rate, self.on_timer)
        self.get_logger().info(
            'imu_sim: profile=%s rate=%.0fHz duration=%.1fs' % (self.profile, self.rate, self.duration))

    def on_timer(self):
        if self.finished:
            return
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        t = now_sec - self._start_sec  # 相对秒数，用于 trajectory
        if t > self.duration:
            # 只置标志，由主循环负责收尾，避免在回调里调用 shutdown 造成阻塞
            self.finished = True
            return

        roll, pitch, yaw = attitude_at(t, self.profile)
        q_true = quat_normalize(quat_from_euler(roll, pitch, yaw))
        gyro = body_rates_at(t, self.profile) + self.bias
        gyro = gyro + self.rng.normal(0.0, self.gyro_noise, 3)
        accel = 9.81 * gravity_in_body(q_true) + self.rng.normal(0.0, self.accel_noise, 3)

        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'imu_link'
        # 没有姿态输出，按 REP 145 用 -1 标记 orientation 不可用
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])
        self.pub_imu.publish(msg)

        gt = QuaternionStamped()
        gt.header = msg.header
        gt.quaternion.x, gt.quaternion.y, gt.quaternion.z, gt.quaternion.w = quat_to_msg(q_true)
        self.pub_gt.publish(gt)

        self.rows.append((now_sec, math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))

    def save(self):
        if self.saved:
            return
        self.saved = True
        os.makedirs(self.out_dir, exist_ok=True)
        path = os.path.join(self.out_dir, '%s_ground_truth.csv' % self.profile)
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'roll_deg', 'pitch_deg', 'yaw_deg'])
            writer.writerows(self.rows)
        self.get_logger().info('已保存 %d 条真值到 %s' % (len(self.rows), path))


def main(args=None):
    rclpy.init(args=args)
    node = ImuSim()
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
