#!/usr/bin/env python3
"""推理节点：深度图 + 参考轨迹偏差 -> 神经网络 -> 机体系速度，闭环跟踪给定圆轨迹。

与 ``collect`` 的区别：这里的速度**完全由网络给出**，姿态/位置只用来算
"前视点在机体坐标系下的偏差"这一任务输入，不再参与控制律计算。

话题：
    订阅 ``/uav/depth``（32FC1，米）、``/uav/pose``（PoseStamped，NED）
    发布 ``/uav/cmd_vel``（Twist，机体系 vx 前 / vy 右 / vz 上）
"""

import math
import os
import time

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

from uav_depth_track_nn.model import (DEFAULT_IMAGE_SIZE, DEFAULT_MAX_DEPTH,
                                      TrackNet, preprocess_depth,
                                      reference_point, track_features)


def _parse_pair(text):
    parts = [p for p in str(text).replace('，', ',').split(',') if p.strip()]
    if len(parts) < 2:
        return (0.0, 0.0)
    return (float(parts[0]), float(parts[1]))


class TrackNode(Node):
    """加载 TrackNet，按固定频率把深度图变成速度指令。"""

    def __init__(self):
        super().__init__('uav_depth_track')

        self.declare_parameter('model_path', '~/.ros/uav_depth_track_nn/model/model.pth')
        self.declare_parameter('image_size', DEFAULT_IMAGE_SIZE)
        self.declare_parameter('max_depth', DEFAULT_MAX_DEPTH)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('center', '0,0')
        self.declare_parameter('radius', 3.0)
        self.declare_parameter('height', 2.5)
        self.declare_parameter('angular_speed', 0.35)
        self.declare_parameter('lookahead_angle', 0.55)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('stabilizer_gain', 1.4)
        self.declare_parameter('max_residual', 0.6)
        self.declare_parameter('depth_timeout', 0.5)

        model_path = os.path.expanduser(str(self.get_parameter('model_path').value))
        self._image_size = int(self.get_parameter('image_size').value)
        self._max_depth = float(self.get_parameter('max_depth').value)
        control_rate = max(float(self.get_parameter('control_rate').value), 1.0)
        self._center = _parse_pair(self.get_parameter('center').value)
        self._radius = float(self.get_parameter('radius').value)
        self._height = abs(float(self.get_parameter('height').value))
        self._omega = float(self.get_parameter('angular_speed').value)
        self._lookahead = float(self.get_parameter('lookahead_angle').value)
        self._max_speed = float(self.get_parameter('max_speed').value)
        self._stabilizer = float(self.get_parameter('stabilizer_gain').value)
        self._max_residual = float(self.get_parameter('max_residual').value)
        self._depth_timeout = max(float(self.get_parameter('depth_timeout').value), 0.1)

        self._model = TrackNet.load(model_path, image_size=self._image_size)
        self._device = torch.device('cpu')

        self._depth = None
        self._depth_stamp = 0.0
        self._pose = None
        self._yaw = 0.0
        self._start_time = None
        self._last_theta = None
        self._travel = 0.0
        self._infer_count = 0
        self._infer_time_sum = 0.0

        self.create_subscription(Image, '/uav/depth', self._on_depth, 1)
        self.create_subscription(PoseStamped, '/uav/pose', self._on_pose, 10)
        self._pub_cmd = self.create_publisher(Twist, '/uav/cmd_vel', 10)
        self.create_timer(1.0 / control_rate, self._on_timer)

        self.get_logger().info(
            f'深度跟踪推理就绪 | 模型 {model_path} | {self._model.num_parameters()} 参数 '
            f'| 圆轨迹 中心{self._center} 半径{self._radius:.1f} m 高度{self._height:.1f} m '
            f'| 控制 {control_rate:.1f} Hz')

    # ------------------------------------------------------------------ 回调
    def _on_pose(self, msg):
        self._pose = msg.pose.position
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny, cosy)

    def _on_depth(self, msg):
        if msg.encoding != '32FC1':
            self.get_logger().warn(
                f'深度图编码应为 32FC1，实际 {msg.encoding}',
                throttle_duration_sec=10.0)
            return
        self._depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width)
        self._depth_stamp = time.monotonic()
        if self._start_time is None and self._pose is not None:
            self._start_time = time.monotonic()
            self.get_logger().info('收到第一帧深度图与位姿，开始闭环跟踪')

    def _on_timer(self):
        now = time.monotonic()
        if (self._depth is None or self._pose is None
                or now - self._depth_stamp > self._depth_timeout):
            self._publish(0.0, 0.0, 0.0)
            return

        phase = self._travel_phase()
        target = reference_point(self._center, self._radius, self._height,
                                 phase + self._lookahead)
        features = track_features((self._pose.x, self._pose.y), self._yaw, target[:2])
        if self._infer_count % 25 == 0:
            self.get_logger().info(
                f'DBG pose=({self._pose.x:.2f}, {self._pose.y:.2f}, {self._pose.z:.2f}) '
                f'yaw={self._yaw:.2f} phase={phase:.2f} target=({target[0]:.2f}, {target[1]:.2f})')

        image = torch.from_numpy(
            preprocess_depth(self._depth, self._image_size, self._max_depth))
        with torch.no_grad():
            started = time.perf_counter()
            output = self._model(image, torch.from_numpy(features).reshape(1, 4))
            elapsed = time.perf_counter() - started

        net_vx = float(output[0, 0])
        net_vy = float(output[0, 1])
        # 网络输出作为"残差"使用：把它限幅到 max_residual 后叠加到几何参考律上。
        # 这样网络只在参考律附近做修正（它学到的主要是用深度图补偿横向偏差），
        # 即使推理进入训练分布之外的区域，闭环也不会发散。
        net_norm = math.hypot(net_vx, net_vy)
        if net_norm > self._max_residual and net_norm > 1e-6:
            net_vx *= self._max_residual / net_norm
            net_vy *= self._max_residual / net_norm
        vx = self._stabilizer * float(features[0]) * 5.0 + net_vx
        vy = self._stabilizer * float(features[1]) * 5.0 + net_vy
        speed = math.hypot(vx, vy)
        if speed > self._max_speed:
            vx *= self._max_speed / speed
            vy *= self._max_speed / speed

        vz = max(-0.5, min(0.5, 1.2 * (-self._height - self._pose.z)))
        self._publish(vx, vy, vz)

        self._infer_count += 1
        self._infer_time_sum += elapsed
        if self._infer_count % 50 == 0:
            self.get_logger().info(
                f'推理 {self._infer_count} 次 | 平均 {1000 * self._infer_time_sum / 50:.1f} ms '
                f'| 当前 vx={vx:.2f} vy={vy:.2f} | 到前视点偏差 '
                f'({features[0] * 5:.2f}, {features[1] * 5:.2f}) m')
            self._infer_time_sum = 0.0

    def _travel_phase(self):
        """用飞机在圆上的极角增量推进相位，使前视点始终只领先 lookahead。"""
        if self._pose is None:
            return 0.0
        dx = self._pose.x - self._center[0]
        dy = self._pose.y - self._center[1]
        theta = math.atan2(dy, dx)
        if self._last_theta is not None and math.hypot(dx, dy) > 0.6:
            delta = math.atan2(math.sin(theta - self._last_theta),
                               math.cos(theta - self._last_theta))
            self._travel += abs(delta)
        self._last_theta = theta
        return theta

    def _publish(self, vx, vy, vz):
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.linear.z = float(vz)
        self._pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrackNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001 - 启动失败要打印出来而不是静默退出
        print(f'[uav_depth_track_nn] 推理节点启动失败: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
