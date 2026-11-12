#!/usr/bin/env python3
"""差速小车里程计闭环精确运动节点。

在键盘遥控基础上增加单键相对运动：
  1/2  前进/后退 0.5 m
  3/4  左/右转 90 度
  5    转 180 度
  6    正方形演示（0.5 m + 90 度，重复四次）
  0/s  取消当前动作
"""

import math
import os
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

ARROW_KEYS = {
    '\x1b[A': 'up',
    '\x1b[B': 'down',
    '\x1b[C': 'right',
    '\x1b[D': 'left',
}


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PrecisionMoves(Node):
    """键盘遥控 + 里程计闭环相对运动。"""

    def __init__(self):
        super().__init__('precision_moves')

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('status_topic', 'precision_moves/status')
        self.declare_parameter('marker_topic', 'precision_moves/markers')
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('distance_step', 0.5)
        self.declare_parameter('turn_angle', 90.0)
        self.declare_parameter('distance_tolerance', 0.01)
        self.declare_parameter('angle_tolerance', 0.02)
        self.declare_parameter('linear_gain', 1.8)
        self.declare_parameter('angular_gain', 2.2)

        self.linear_speed = max(
            0.05, float(self.get_parameter('linear_speed').value))
        self.angular_speed = max(
            0.1, float(self.get_parameter('angular_speed').value))
        self.publish_rate = max(
            1.0, float(self.get_parameter('publish_rate').value))
        self.distance_step = max(
            0.05, float(self.get_parameter('distance_step').value))
        self.turn_angle = math.radians(float(
            self.get_parameter('turn_angle').value))
        self.distance_tolerance = max(
            0.002, float(self.get_parameter('distance_tolerance').value))
        self.angle_tolerance = max(
            0.005, float(self.get_parameter('angle_tolerance').value))
        self.linear_gain = max(
            0.1, float(self.get_parameter('linear_gain').value))
        self.angular_gain = max(
            0.1, float(self.get_parameter('angular_gain').value))

        cmd_topic = str(self.get_parameter('cmd_vel_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)

        self.cmd_publisher = self.create_publisher(Twist, cmd_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, marker_topic, 10)
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Odometry, odom_topic, self._on_odom, odom_qos)

        self._pose: Optional[Pose2D] = None
        self._last_yaw: Optional[float] = None
        self._yaw_accum = 0.0
        self._mode = 'manual'  # manual / move / turn
        self._start_pose: Optional[Pose2D] = None
        self._move_target = 0.0
        self._turn_target = 0.0
        self._queue: List[Tuple[str, float]] = []
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._running = True
        self._last_status = ''

        self._stdin_fd = sys.stdin.fileno()
        self._original_term_settings = None
        self._key_buffer = b''
        if not sys.stdin.isatty():
            raise RuntimeError(
                '标准输入不是交互式终端，无法读取按键。'
                '请在终端中直接运行本节点。'
            )

    # ------------------------------------------------------------------
    # 回调与运动控制
    # ------------------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = normalize_angle(math.atan2(siny, cosy))
        self._pose = Pose2D(p.x, p.y, yaw)
        if self._last_yaw is not None and self._mode == 'turn':
            self._yaw_accum += normalize_angle(yaw - self._last_yaw)
        self._last_yaw = yaw

    def _publish_status(self, text: str):
        self._last_status = text
        msg = String()
        msg.data = text
        self.status_publisher.publish(msg)

    def _start_next(self):
        if not self._queue:
            self._mode = 'manual'
            self._cmd_linear = 0.0
            self._cmd_angular = 0.0
            self._publish_status('manual')
            return
        kind, value = self._queue.pop(0)
        if self._pose is None:
            self.get_logger().warn('尚未收到 /odom，无法开始精确运动')
            self._queue.clear()
            self._mode = 'manual'
            self._publish_status('manual')
            return
        self._start_pose = self._pose
        self._last_yaw = self._pose.yaw
        self._yaw_accum = 0.0
        if kind == 'move':
            self._mode = 'move'
            self._move_target = value
            self._publish_status(f'move {value:.2f} m')
            self.get_logger().info(f'开始相对运动：{value:.2f} m')
        else:
            self._mode = 'turn'
            self._turn_target = value
            self._publish_status(f'turn {math.degrees(value):.0f} deg')
            self.get_logger().info(
                f'开始相对转向：{math.degrees(value):.1f} 度')

    def _start_single(self, kind: str, value: float):
        self._queue = [(kind, value)]
        self._start_next()

    def _finish_current(self):
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        if self._mode == 'move' and self._pose is not None:
            yaw0 = self._start_pose.yaw
            dx = self._pose.x - self._start_pose.x
            dy = self._pose.y - self._start_pose.y
            traveled = math.cos(yaw0) * dx + math.sin(yaw0) * dy
            self.get_logger().info(
                f'相对运动完成：目标 {self._move_target:.3f} m，'
                f'实际 {traveled:.3f} m，误差 '
                f'{traveled - self._move_target:+.4f} m')
        elif self._mode == 'turn':
            self.get_logger().info(
                f'相对转向完成：目标 {math.degrees(self._turn_target):.1f} 度，'
                f'实际 {math.degrees(self._yaw_accum):.1f} 度，误差 '
                f'{math.degrees(self._yaw_accum - self._turn_target):+.2f} 度')
        self._start_next()

    def _update_control(self):
        if self._pose is None or self._start_pose is None:
            return

        if self._mode == 'move':
            yaw0 = self._start_pose.yaw
            dx = self._pose.x - self._start_pose.x
            dy = self._pose.y - self._start_pose.y
            traveled = math.cos(yaw0) * dx + math.sin(yaw0) * dy
            remaining = self._move_target - traveled
            if abs(remaining) <= self.distance_tolerance:
                self._finish_current()
                return
            heading_error = normalize_angle(yaw0 - self._pose.yaw)
            if abs(heading_error) > 0.5:
                self._cmd_linear = 0.0
                self._cmd_angular = math.copysign(
                    self.angular_speed, heading_error)
                return
            speed = min(self.linear_speed,
                        self.linear_gain * abs(remaining))
            speed = max(0.04, speed)
            if remaining < 0.0:
                speed = -speed
            self._cmd_linear = speed
            self._cmd_angular = max(
                -0.5 * self.angular_speed,
                min(0.5 * self.angular_speed,
                    self.angular_gain * heading_error))

        elif self._mode == 'turn':
            remaining = self._turn_target - self._yaw_accum
            if abs(remaining) <= self.angle_tolerance:
                self._finish_current()
                return
            angular = min(self.angular_speed,
                          max(0.08, self.angular_gain * abs(remaining)))
            self._cmd_linear = 0.0
            self._cmd_angular = math.copysign(angular, remaining)

    # ------------------------------------------------------------------
    # 键盘
    # ------------------------------------------------------------------
    def _enable_cbreak(self):
        self._original_term_settings = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)

    def _restore_terminal(self):
        if self._original_term_settings is not None:
            termios.tcsetattr(
                self._stdin_fd, termios.TCSADRAIN,
                self._original_term_settings)
            self._original_term_settings = None

    def _parse_keys(self, data: bytes):
        self._key_buffer += data
        keys = []
        while self._key_buffer:
            if self._key_buffer.startswith(b'\x1b'):
                if len(self._key_buffer) < 3:
                    break
                seq = self._key_buffer[:3].decode('ascii', errors='ignore')
                self._key_buffer = self._key_buffer[3:]
                keys.append(ARROW_KEYS.get(seq, seq))
            else:
                key = self._key_buffer[:1].decode('utf-8', errors='ignore')
                self._key_buffer = self._key_buffer[1:]
                keys.append(key)
        return keys

    def _read_keys(self, timeout: float):
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return []
        data = os.read(self._stdin_fd, 64)
        if not data:
            return ['q']
        return self._parse_keys(data)

    def _cancel(self):
        self._queue.clear()
        self._mode = 'manual'
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._publish_status('manual')
        self.get_logger().info('已取消当前精确动作')

    def _manual(self, linear: float, angular: float):
        if self._mode != 'manual':
            self._cancel()
        self._cmd_linear = linear
        self._cmd_angular = angular

    def _handle_key(self, key: str) -> bool:
        if key in ('q', 'Q', '\x03'):
            return False
        if key == '1':
            self._start_single('move', self.distance_step)
        elif key == '2':
            self._start_single('move', -self.distance_step)
        elif key == '3':
            self._start_single('turn', self.turn_angle)
        elif key == '4':
            self._start_single('turn', -self.turn_angle)
        elif key == '5':
            self._start_single('turn', math.pi)
        elif key == '6':
            self._queue = []
            for _ in range(4):
                self._queue.append(('move', self.distance_step))
                self._queue.append(('turn', self.turn_angle))
            self._start_next()
        elif key in ('0', 's', 'S', ' ', 'k', 'K'):
            self._cancel()
        elif key in ('w', 'W', 'up'):
            self._manual(self.linear_speed, 0.0)
        elif key in ('x', 'X', 'down'):
            self._manual(-self.linear_speed, 0.0)
        elif key in ('a', 'A', 'left'):
            self._manual(0.0, self.angular_speed)
        elif key in ('d', 'D', 'right'):
            self._manual(0.0, -self.angular_speed)
        elif key in ('+', '='):
            self.linear_speed = min(2.0, self.linear_speed + 0.05)
        elif key in ('-', '_'):
            self.linear_speed = max(0.05, self.linear_speed - 0.05)
        return True

    def print_usage(self):
        self.get_logger().info('差速小车精确运动节点已启动')
        self.get_logger().info('  1/2 : 前进/后退 0.5 m')
        self.get_logger().info('  3/4 : 左/右转 90 度')
        self.get_logger().info('  5   : 转 180 度')
        self.get_logger().info('  6   : 正方形轨迹演示')
        self.get_logger().info('  0/s : 取消当前动作')
        self.get_logger().info('  w/x/a/d 或方向键 : 手动驾驶')
        self.get_logger().info('  q/Ctrl-C : 退出')

    # ------------------------------------------------------------------
    # 发布
    # ------------------------------------------------------------------
    def _publish_cmd(self):
        msg = Twist()
        msg.linear.x = self._cmd_linear
        msg.angular.z = self._cmd_angular
        self.cmd_publisher.publish(msg)

    def _publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = stamp
        marker.ns = 'precision_moves'
        marker.id = 0
        if self._mode == 'manual':
            marker.action = Marker.DELETE
        else:
            marker.action = Marker.ADD
            marker.type = Marker.ARROW
            marker.pose.position.x = self._pose.x if self._pose else 0.0
            marker.pose.position.y = self._pose.y if self._pose else 0.0
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 1.0
        array = MarkerArray()
        array.markers.append(marker)
        self.marker_publisher.publish(array)

    def stop_robot(self):
        msg = Twist()
        for _ in range(3):
            try:
                self.cmd_publisher.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.02)
            except Exception:
                break

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def run(self):
        self._enable_cbreak()
        self.print_usage()
        period = 1.0 / self.publish_rate
        self._publish_status('manual')

        try:
            while rclpy.ok() and self._running:
                loop_start = time.monotonic()
                for key in self._read_keys(period):
                    if not self._handle_key(key):
                        self._running = False
                        break
                rclpy.spin_once(self, timeout_sec=0.0)
                self._update_control()
                self._publish_cmd()
                self._publish_markers()
                elapsed = time.monotonic() - loop_start
                if elapsed < period:
                    time.sleep(period - elapsed)
        except (KeyboardInterrupt, EOFError):
            pass
        finally:
            self._restore_terminal()
            self._cmd_linear = 0.0
            self._cmd_angular = 0.0
            self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PrecisionMoves()
        node.run()
    except RuntimeError as exc:
        print(f'[precision_moves] 错误: {exc}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
