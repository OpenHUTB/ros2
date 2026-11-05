# Copyright 2026 OpenHUTB
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See http://www.apache.org/licenses/LICENSE-2.0 for the full text.
"""路径跟踪控制节点：纯追踪（pure pursuit）+ 高度保持 + 航向控制。

订阅 ``/plan`` 与 ``/uav/pose``，按纯追踪律算出期望世界系速度，
再旋转到机体系发布 ``/uav/cmd_vel``（``airsim_source`` 会把它变成
``moveByVelocityBodyFrameAsync``）。``/nav_status`` 用字符串播报当前状态。
"""

import math

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String

from uav_slam_nav.grid_utils import clamp, wrap_angle, yaw_from_quat


class Controller(Node):
    """纯追踪控制器。"""

    def __init__(self):
        super().__init__('controller', namespace='uav')

        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('lookahead', 1.5)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('min_speed', 0.3)
        self.declare_parameter('goal_tolerance', 0.6)
        self.declare_parameter('cruise_height', 3.0)
        self.declare_parameter('kp_altitude', 1.0)
        self.declare_parameter('max_vertical_speed', 1.0)
        self.declare_parameter('kp_yaw', 1.5)
        self.declare_parameter('max_yaw_rate', 1.0)
        self.declare_parameter('slowdown_distance', 3.0)

        self._rate = max(1.0, float(self.get_parameter('control_rate').value))
        self._lookahead = max(0.2, float(self.get_parameter('lookahead').value))
        self._max_speed = float(self.get_parameter('max_speed').value)
        self._min_speed = float(self.get_parameter('min_speed').value)
        self._goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self._cruise_height = float(self.get_parameter('cruise_height').value)
        self._kp_alt = float(self.get_parameter('kp_altitude').value)
        self._max_vz = float(self.get_parameter('max_vertical_speed').value)
        self._kp_yaw = float(self.get_parameter('kp_yaw').value)
        self._max_yaw = float(self.get_parameter('max_yaw_rate').value)
        self._slowdown = max(0.1, float(self.get_parameter('slowdown_distance').value))

        self._path = []          # [(x, y), ...] 世界系
        self._index = 0
        self._pose = None        # (x, y, z, yaw)
        self._status = ''

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._status_pub = self.create_publisher(String, '/nav_status', 10)
        self.create_subscription(Path, '/plan', self._on_path, 5)
        self.create_subscription(PoseStamped, 'pose', self._on_pose, 10)

        self.create_timer(1.0 / self._rate, self._step)
        self.get_logger().info(
            f'控制节点启动：前视距离 {self._lookahead} m，最大速度 {self._max_speed} m/s，'
            f'巡航高度 {self._cruise_height} m')

    # ------------------------------------------------------------------ 订阅
    def _on_path(self, msg):
        """接收新路径，重置跟踪索引。"""
        self._path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._index = 0
        if self._path:
            self._set_status(f'tracking {len(self._path)} waypoints')
        else:
            self._set_status('idle')

    def _on_pose(self, msg):
        """缓存位姿：位置 + 偏航角（NED 世界系）。"""
        quat = msg.pose.orientation
        yaw = yaw_from_quat(quat.x, quat.y, quat.z, quat.w)
        self._pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            yaw,
        )

    def _set_status(self, text):
        """状态变化时才发布，避免刷屏。"""
        if text == self._status:
            return
        self._status = text
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(f'导航状态：{text}')

    # ------------------------------------------------------------------ 控制
    def _step(self):
        """每个控制周期算一次速度指令。"""
        if self._pose is None:
            return
        x, y, z, yaw = self._pose
        height = -z  # NED 的 z 向下，取负得到离地高度

        if not self._path:
            self._publish_cmd(0.0, 0.0, self._vertical_speed(height), 0.0)
            return

        goal_x, goal_y = self._path[-1]
        if math.hypot(goal_x - x, goal_y - y) < self._goal_tolerance:
            self._publish_cmd(0.0, 0.0, self._vertical_speed(height), 0.0)
            self._set_status('arrived')
            return

        target = self._lookahead_point(x, y)
        if target is None:
            self._publish_cmd(0.0, 0.0, self._vertical_speed(height), 0.0)
            self._set_status('path finished')
            return

        dx = target[0] - x
        dy = target[1] - y
        distance = math.hypot(dx, dy)
        if distance < 1e-3:
            return

        # 接近终点时线性降速，避免冲过头
        remaining = math.hypot(goal_x - x, goal_y - y)
        speed = self._max_speed * clamp(remaining / self._slowdown, 0.0, 1.0)
        speed = max(self._min_speed, speed)

        world_vx = speed * dx / distance
        world_vy = speed * dy / distance
        body_vx, body_vy = self._world_to_body(world_vx, world_vy, yaw)

        heading_error = wrap_angle(math.atan2(world_vy, world_vx) - yaw)
        yaw_rate = clamp(self._kp_yaw * heading_error, -self._max_yaw, self._max_yaw)

        self._publish_cmd(body_vx, body_vy, self._vertical_speed(height), yaw_rate)

    def _lookahead_point(self, x, y):
        """沿路径找距离当前点不小于前视距离的第一个点。"""
        while self._index < len(self._path) - 1:
            point = self._path[self._index]
            if math.hypot(point[0] - x, point[1] - y) >= self._lookahead:
                return point
            self._index += 1
        return self._path[-1] if self._path else None

    def _vertical_speed(self, height):
        """高度保持：P 控制，NED 下向下为正，所以取负号。"""
        error = self._cruise_height - height
        # 死区：几厘米的高度抖动不该让飞机一直上下打摆
        if abs(error) < 0.25:
            return 0.0
        return clamp(-self._kp_alt * error, -self._max_vz, self._max_vz)

    @staticmethod
    def _world_to_body(vx, vy, yaw):
        """世界系速度 -> 机体系速度（x 前，y 右）。"""
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        body_x = vx * cos_yaw + vy * sin_yaw
        body_y = -vx * sin_yaw + vy * cos_yaw
        return body_x, body_y

    def _publish_cmd(self, vx, vy, vz, yaw_rate):
        """发布 Twist，linear 为机体系速度，angular.z 为偏航角速度。"""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(yaw_rate)
        self._cmd_pub.publish(msg)
