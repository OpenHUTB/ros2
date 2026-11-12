#!/usr/bin/env python3
"""差速小车定速巡航与航向保持节点。

手动模式用键盘驾驶；按 c 进入巡航后，节点用 /odom 闭环 PID
维持目标线速度，并用偏航角反馈保持目标航向。
"""

import math
import os
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String

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


class CruiseControl(Node):
    """键盘遥控 + 里程计闭环定速巡航与航向保持。"""

    def __init__(self):
        super().__init__('cruise_control')

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('status_topic', 'cruise_control/status')
        self.declare_parameter('state_topic', 'cruise_control/state')
        self.declare_parameter('cruise_speed', 0.3)
        self.declare_parameter('max_speed', 0.40)
        self.declare_parameter('manual_speed', 0.25)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('kp_speed', 0.5)
        self.declare_parameter('ki_speed', 0.0)
        self.declare_parameter('kd_speed', 0.02)
        self.declare_parameter('kp_yaw', 1.8)
        self.declare_parameter('kd_yaw', 0.15)
        self.declare_parameter('heading_step', 0.08)
        self.declare_parameter('publish_rate', 20.0)

        self.cruise_speed = max(
            0.05, float(self.get_parameter('cruise_speed').value))
        self.max_speed = max(
            self.cruise_speed, float(self.get_parameter('max_speed').value))
        self.manual_speed = max(
            0.05, float(self.get_parameter('manual_speed').value))
        self.max_angular = max(
            0.1, float(self.get_parameter('max_angular').value))
        self.kp_speed = float(self.get_parameter('kp_speed').value)
        self.ki_speed = float(self.get_parameter('ki_speed').value)
        self.kd_speed = float(self.get_parameter('kd_speed').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.kd_yaw = float(self.get_parameter('kd_yaw').value)
        self.heading_step = max(
            0.01, float(self.get_parameter('heading_step').value))
        self.publish_rate = max(
            1.0, float(self.get_parameter('publish_rate').value))

        cmd_topic = str(self.get_parameter('cmd_vel_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        state_topic = str(self.get_parameter('state_topic').value)

        self.cmd_publisher = self.create_publisher(Twist, cmd_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.state_publisher = self.create_publisher(
            Float32MultiArray, state_topic, 10)

        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Odometry, odom_topic, self._on_odom, odom_qos)

        self._pose: Optional[Pose2D] = None
        self._v = 0.0
        self._w = 0.0
        self._mode = 'manual'  # manual / cruise
        self._target_speed = self.cruise_speed
        self._target_yaw = 0.0
        self._integral = 0.0
        self._prev_speed_error = 0.0
        self._prev_yaw_error = 0.0
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
    # 回调与状态
    # ------------------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = normalize_angle(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        ))
        self._pose = Pose2D(p.x, p.y, yaw)
        self._v = msg.twist.twist.linear.x
        self._w = msg.twist.twist.angular.z

    def _publish_status(self, text: str):
        if text == self._last_status:
            return
        self._last_status = text
        msg = String()
        msg.data = text
        self.status_publisher.publish(msg)

    def _start_cruise(self):
        if self._pose is None:
            self.get_logger().warn('尚未收到 /odom，无法启动巡航')
            return
        self._mode = 'cruise'
        self._target_speed = self.cruise_speed
        self._target_yaw = self._pose.yaw
        self._integral = 0.0
        self._prev_speed_error = 0.0
        self._prev_yaw_error = 0.0
        self._publish_status(f'cruise {self._target_speed:.2f} m/s')
        self.get_logger().info(
            f'进入定速巡航：目标 {self._target_speed:.2f} m/s，'
            f'保持航向 {math.degrees(self._target_yaw):.1f} 度')

    def _stop_cruise(self):
        self._mode = 'manual'
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._publish_status('manual')
        self.get_logger().info('退出定速巡航')

    def _nudge_heading(self, delta: float):
        self._target_yaw = normalize_angle(self._target_yaw + delta)
        self.get_logger().info(
            f'目标航向调整为 {math.degrees(self._target_yaw):.1f} 度')

    def _update_cruise(self, dt: float):
        if self._mode != 'cruise' or self._pose is None:
            return

        speed_error = self._target_speed - self._v
        derivative = 0.0
        if dt > 1e-6:
            derivative = (speed_error - self._prev_speed_error) / dt
        self._prev_speed_error = speed_error

        # 前馈 + PI(D)，并对输出饱和时的积分做条件更新，避免冲过目标速度。
        integral_candidate = max(
            -0.2, min(0.2, self._integral + speed_error * dt))
        linear_candidate = (
            self._target_speed
            + self.kp_speed * speed_error
            + self.ki_speed * integral_candidate
            + self.kd_speed * derivative
        )
        if 0.0 <= linear_candidate <= self.max_speed:
            self._integral = integral_candidate
        linear = max(0.0, min(self.max_speed, linear_candidate))

        yaw_error = normalize_angle(self._target_yaw - self._pose.yaw)
        yaw_derivative = 0.0
        if dt > 1e-6:
            yaw_derivative = (yaw_error - self._prev_yaw_error) / dt
        self._prev_yaw_error = yaw_error
        angular = (
            self.kp_yaw * yaw_error
            + self.kd_yaw * yaw_derivative
        )
        angular = max(-self.max_angular, min(self.max_angular, angular))

        # 转向误差较大时降低线速度，避免巡航中甩头。
        if abs(yaw_error) > 0.5:
            linear *= 0.35
        elif abs(yaw_error) > 0.2:
            linear *= 0.7

        self._cmd_linear = linear
        self._cmd_angular = angular

    def _publish_state(self):
        msg = Float32MultiArray()
        yaw = self._pose.yaw if self._pose else 0.0
        msg.data = [
            float(self._target_speed),
            float(self._target_yaw),
            float(self._v),
            float(yaw),
            float(self._target_speed - self._v),
            float(normalize_angle(self._target_yaw - yaw)),
        ]
        self.state_publisher.publish(msg)

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

    def _manual(self, linear: float, angular: float):
        if self._mode == 'cruise':
            self._stop_cruise()
        self._cmd_linear = linear
        self._cmd_angular = angular

    def _handle_key(self, key: str) -> bool:
        if key in ('q', 'Q', '\x03'):
            return False
        if key in ('c', 'C'):
            if self._mode == 'cruise':
                self._stop_cruise()
            else:
                self._start_cruise()
        elif key in ('s', 'S', ' ', 'k', 'K'):
            self._cmd_linear = 0.0
            self._cmd_angular = 0.0
            if self._mode == 'cruise':
                self._stop_cruise()
        elif key in ('a', 'A', 'left', 'd', 'D', 'right'):
            if self._mode == 'cruise':
                delta = self.heading_step
                if key in ('a', 'A', 'left'):
                    self._nudge_heading(delta)
                else:
                    self._nudge_heading(-delta)
            else:
                if key in ('a', 'A', 'left'):
                    self._manual(0.0, self.max_angular)
                else:
                    self._manual(0.0, -self.max_angular)
        elif key in ('w', 'W', 'up'):
            self._manual(self.manual_speed, 0.0)
        elif key in ('x', 'X', 'down'):
            self._manual(-self.manual_speed, 0.0)
        elif key in ('+', '='):
            if self._mode == 'cruise':
                self._target_speed = min(
                    self.max_speed, self._target_speed + 0.05)
                self._publish_status(
                    f'cruise {self._target_speed:.2f} m/s')
            else:
                self.manual_speed = min(1.0, self.manual_speed + 0.05)
        elif key in ('-', '_'):
            if self._mode == 'cruise':
                self._target_speed = max(
                    0.05, self._target_speed - 0.05)
                self._publish_status(
                    f'cruise {self._target_speed:.2f} m/s')
            else:
                self.manual_speed = max(0.05, self.manual_speed - 0.05)
        return True

    def print_usage(self):
        self.get_logger().info('差速小车定速巡航/航向保持已启动')
        self.get_logger().info('  c : 进入/退出定速巡航')
        self.get_logger().info('  巡航中 a/d : 左/右微调目标航向')
        self.get_logger().info('  巡航中 +/- : 调整目标速度')
        self.get_logger().info('  w/x/a/d : 手动驾驶（会自动退出巡航）')
        self.get_logger().info('  s/空格 : 停止')
        self.get_logger().info('  q/Ctrl-C : 退出')

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def _publish_cmd(self):
        msg = Twist()
        msg.linear.x = self._cmd_linear
        msg.angular.z = self._cmd_angular
        self.cmd_publisher.publish(msg)

    def stop_robot(self):
        msg = Twist()
        for _ in range(3):
            try:
                self.cmd_publisher.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.02)
            except Exception:
                break

    def run(self):
        self._enable_cbreak()
        self.print_usage()
        period = 1.0 / self.publish_rate
        last_time = time.monotonic()
        self._publish_status('manual')

        try:
            while rclpy.ok() and self._running:
                loop_start = time.monotonic()
                dt = min(0.2, loop_start - last_time)
                last_time = loop_start
                for key in self._read_keys(period):
                    if not self._handle_key(key):
                        self._running = False
                        break
                rclpy.spin_once(self, timeout_sec=0.0)
                if self._mode == 'cruise':
                    self._update_cruise(dt)
                self._publish_cmd()
                self._publish_state()
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
        node = CruiseControl()
        node.run()
    except RuntimeError as exc:
        print(f'[cruise_control] 错误: {exc}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
