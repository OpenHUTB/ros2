#!/usr/bin/env python3
"""
任务 4 —— 端到端视觉伺服 ROS2 节点
====================================
订阅: /rov/camera/image_raw  (sensor_msgs/Image, 前视相机)
发布: /cmd_vel               (geometry_msgs/Twist)

策略网络完全由图像输出 [vx, vy, wz]；本节点只负责:
  1. 图像 -> NumPy 转换；
  2. 调用 VisuomotorController 推理；
  3. 航向安全保护（|yaw| > 25° 时优先回正）；
  4. 抵达管线三通汇接点后悬停。

启动:
    ros2 launch rov_mujoco task4.launch.py
"""

import math
import os
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

try:
    from rov_mujoco.end_to_end_networks import VisuomotorController
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from rov_mujoco.end_to_end_networks import VisuomotorController

PIPE_Y = 3.2
GOAL_X = 3.0


class EndToEndNode(Node):
    def __init__(self):
        super().__init__('end_to_end_node')

        self.declare_parameter('architecture', 'nature_cnn')
        self.declare_parameter('algorithm', 'bc')
        self.declare_parameter('weights_path', '')
        self.declare_parameter('nominal_speed', 0.45)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('goal_tolerance', 0.40)

        arch = self.get_parameter('architecture').value
        algo = self.get_parameter('algorithm').value
        weights = self.get_parameter('weights_path').value or None
        self.nominal_speed = float(self.get_parameter('nominal_speed').value)
        self.goal_tol = float(self.get_parameter('goal_tolerance').value)

        self.controller = VisuomotorController(
            architecture=arch, algorithm=algo, weights_path=weights,
            nominal_speed=self.nominal_speed)
        self.get_logger().info(
            f"视觉策略: {arch} / {algo} | 权值={self.controller.weights_source} | "
            f"设备={self.controller.device}")

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_image = self.create_subscription(
            Image, '/rov/camera/image_raw', self._image_callback, 10)

        self.image = None
        self.last_latency = 0.0
        self.completed = False
        self.completion_logged = False
        self.cycles = 0
        period = 1.0 / max(1e-3, float(self.get_parameter('control_rate').value))
        self.timer = self.create_timer(period, self._control_callback)

    def _image_callback(self, msg):
        try:
            channels = 3 if msg.encoding.lower() in ('rgb8', 'bgr8') else 1
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape(msg.height, msg.width, channels)
            if channels == 3 and msg.encoding.lower() == 'bgr8':
                img = img[..., ::-1]
            self.image = img.copy()
        except Exception as exc:
            self.get_logger().warn(f"图像解析失败: {exc}")

    def _control_callback(self):
        if self.image is None:
            return
        cmd = Twist()
        pos_msg = None

        result = self.controller.predict_action(self.image)
        self.last_latency = result['latency_ms']

        # 位姿从仿真全局位姿话题获取（用于终点判定与航向保护）
        yaw = 0.0
        try:
            tf = self._get_pose()
            if tf is not None:
                pos_msg, yaw = tf
        except Exception:
            pass

        if pos_msg is not None:
            dist_goal = math.hypot(GOAL_X - pos_msg[0], PIPE_Y - pos_msg[1])
            if pos_msg[0] >= GOAL_X - 0.15 and dist_goal < self.goal_tol:
                self.completed = True
            if self.completed:
                if not self.completion_logged:
                    self.completion_logged = True
                    self.get_logger().info(
                        f"已抵达管线三通 | x={pos_msg[0]:.2f} y={pos_msg[1]:.2f} "
                        f"| 转入动力定位悬停")
                cmd.linear.x = float(np.clip(1.2 * (GOAL_X - 0.05 - pos_msg[0]),
                                             -0.30, 0.30))
                cmd.linear.y = float(np.clip(1.2 * (PIPE_Y - pos_msg[1]),
                                             -0.30, 0.30))
                cmd.angular.z = float(np.clip(-1.2 * yaw, -0.35, 0.35))
                self.pub_cmd.publish(cmd)
                return

        cmd.linear.x = result['vx']
        cmd.linear.y = result['vy']
        cmd.linear.z = 0.0
        if abs(yaw) > 0.25:
            cmd.angular.z = float(np.clip(-1.8 * yaw, -0.6, 0.6))
        else:
            cmd.angular.z = result['wz']
        self.pub_cmd.publish(cmd)

        self.cycles += 1
        if self.cycles % 25 == 0:
            pose_txt = (""
                        if pos_msg is None else
                        f" | x={pos_msg[0]:.2f} y={pos_msg[1]:.2f} yaw={yaw:+.2f}")
            self.get_logger().info(
                f"推理 {self.last_latency:.1f} ms | vx={cmd.linear.x:.2f} "
                f"vy={cmd.linear.y:+.2f} wz={cmd.angular.z:+.2f}"
                f" | img_mean={float(self.image.mean()):.1f}{pose_txt}")

    def _get_pose(self):
        """从 /rov/odom 读取 (x, y, yaw)"""
        if not hasattr(self, '_odom'):
            from nav_msgs.msg import Odometry
            self._odom = None

            def cb(msg):
                q = msg.pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self._odom = (msg.pose.pose.position.x,
                              msg.pose.pose.position.y, yaw)
            self.create_subscription(Odometry, '/rov/odom', cb, 10)
        if self._odom is None:
            return None
        x, y, yaw = self._odom
        return (x, y), yaw


def main(args=None):
    rclpy.init(args=args)
    node = EndToEndNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
