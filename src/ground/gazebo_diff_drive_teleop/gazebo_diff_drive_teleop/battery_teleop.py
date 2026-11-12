#!/usr/bin/env python3
"""带电量管理与低电量自动返航的差速小车键盘遥控节点。

手动模式：键盘驾驶小车，电量按行驶距离和转向消耗。
低电量：电量低于阈值后自动开回充电点，到点后原地充电。
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
from sensor_msgs.msg import BatteryState
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


class BatteryTeleop(Node):
    """键盘遥控 + 电量模拟 + 低电量返航。"""

    def __init__(self):
        super().__init__('battery_teleop')

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('battery_topic', 'battery_state')
        self.declare_parameter('status_topic', 'battery_return/status')
        self.declare_parameter('marker_topic', 'battery_return/markers')
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('battery_publish_rate', 1.0)
        self.declare_parameter('initial_battery', 100.0)
        self.declare_parameter('low_battery_threshold', 25.0)
        self.declare_parameter('charged_threshold', 80.0)
        self.declare_parameter('drain_per_meter', 8.0)
        self.declare_parameter('drain_per_radian', 2.0)
        self.declare_parameter('idle_drain_per_second', 0.15)
        self.declare_parameter('charge_rate', 8.0)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_yaw', 0.0)
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('yaw_tolerance', 0.12)
        self.declare_parameter('return_linear_gain', 1.2)
        self.declare_parameter('return_angular_gain', 2.0)

        self.linear_speed = max(
            0.05, float(self.get_parameter('linear_speed').value))
        self.angular_speed = max(
            0.1, float(self.get_parameter('angular_speed').value))
        self.publish_rate = max(
            1.0, float(self.get_parameter('publish_rate').value))
        self.battery_publish_rate = max(
            0.2, float(self.get_parameter('battery_publish_rate').value))
        self.battery = min(
            100.0, max(0.0, float(self.get_parameter('initial_battery').value)))
        self.low_threshold = max(
            0.0, float(self.get_parameter('low_battery_threshold').value))
        self.charged_threshold = min(
            100.0, max(self.low_threshold,
                       float(self.get_parameter('charged_threshold').value)))
        self.drain_per_meter = max(
            0.0, float(self.get_parameter('drain_per_meter').value))
        self.drain_per_radian = max(
            0.0, float(self.get_parameter('drain_per_radian').value))
        self.idle_drain = max(
            0.0, float(self.get_parameter('idle_drain_per_second').value))
        self.charge_rate = max(
            0.0, float(self.get_parameter('charge_rate').value))
        self.home = Pose2D(
            float(self.get_parameter('home_x').value),
            float(self.get_parameter('home_y').value),
            float(self.get_parameter('home_yaw').value),
        )
        self.goal_tolerance = max(
            0.02, float(self.get_parameter('goal_tolerance').value))
        self.yaw_tolerance = max(
            0.02, float(self.get_parameter('yaw_tolerance').value))
        self.return_linear_gain = max(
            0.1, float(self.get_parameter('return_linear_gain').value))
        self.return_angular_gain = max(
            0.1, float(self.get_parameter('return_angular_gain').value))

        cmd_topic = str(self.get_parameter('cmd_vel_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        battery_topic = str(self.get_parameter('battery_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)

        self.cmd_publisher = self.create_publisher(Twist, cmd_topic, 10)
        self.battery_publisher = self.create_publisher(
            BatteryState, battery_topic, 10)
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
        self._odom_v = 0.0
        self._odom_w = 0.0
        self._mode = 'manual'  # manual / returning / charging
        self._auto_return_armed = True
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._last_battery_log = self.battery
        self._last_battery_publish = 0.0
        self._last_status = ''
        self._running = True

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
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._pose = Pose2D(p.x, p.y, normalize_angle(math.atan2(siny, cosy)))
        self._odom_v = msg.twist.twist.linear.x
        self._odom_w = msg.twist.twist.angular.z

    def _set_mode(self, mode: str):
        if mode != self._mode:
            self._mode = mode
            self._publish_status(mode)

    def _publish_status(self, text: str):
        if text == self._last_status:
            return
        self._last_status = text
        msg = String()
        msg.data = text
        self.status_publisher.publish(msg)

    def _update_battery(self, dt: float):
        if self._mode == 'charging':
            self.battery = min(100.0, self.battery + self.charge_rate * dt)
            if self.battery >= self.charged_threshold:
                self._set_mode('manual')
                self._auto_return_armed = True
                self.get_logger().info(
                    f'充电完成，电量 {self.battery:.1f}%，恢复手动模式')
            return

        drain = (
            self.idle_drain
            + self.drain_per_meter * abs(self._odom_v)
            + self.drain_per_radian * abs(self._odom_w)
        ) * dt
        self.battery = max(0.0, self.battery - drain)

        if self.battery <= self.low_threshold and self._auto_return_armed:
            self._auto_return_armed = False
            self._set_mode('returning')
            self.get_logger().warn(
                f'电量低（{self.battery:.1f}%），自动返回充电点')

    def _update_return(self):
        if self._mode != 'returning' or self._pose is None:
            return

        dx = self.home.x - self._pose.x
        dy = self.home.y - self._pose.y
        distance = math.hypot(dx, dy)

        if distance > self.goal_tolerance:
            desired = math.atan2(dy, dx)
            heading_error = normalize_angle(desired - self._pose.yaw)
            if abs(heading_error) > 0.8:
                self._cmd_linear = 0.0
                self._cmd_angular = math.copysign(
                    self.angular_speed, heading_error)
            else:
                linear = min(
                    self.linear_speed,
                    self.return_linear_gain * distance,
                )
                if distance < 0.35:
                    linear = min(linear, 0.06)
                self._cmd_linear = linear
                self._cmd_angular = max(
                    -self.angular_speed,
                    min(self.angular_speed,
                        self.return_angular_gain * heading_error),
                )
            return

        yaw_error = normalize_angle(self.home.yaw - self._pose.yaw)
        if abs(yaw_error) > self.yaw_tolerance:
            self._cmd_linear = 0.0
            self._cmd_angular = max(
                -self.angular_speed,
                min(self.angular_speed,
                    self.return_angular_gain * yaw_error),
            )
            return

        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._set_mode('charging')
        self.get_logger().info(
            f'已到达充电点（误差 {distance:.3f} m），开始充电')

    def _stop_motion(self):
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        if self._mode == 'returning':
            self._set_mode('manual')
            self._auto_return_armed = False
            self.get_logger().info('已暂停自动返航；按 h 可重新返航')
        elif self._mode == 'charging':
            self._set_mode('manual')
            self.get_logger().info('已离开充电模式')

    def _manual_override(self, linear: float, angular: float):
        if self._mode != 'manual':
            self._set_mode('manual')
            self._auto_return_armed = False
            self.get_logger().info('手动接管，自动返航已暂停')
        self._cmd_linear = linear
        self._cmd_angular = angular

    def _start_return(self):
        if self._mode == 'charging':
            self.get_logger().info('正在充电，无需返航')
            return
        self._auto_return_armed = True
        self._set_mode('returning')
        self.get_logger().info('开始返回充电点')

    def _force_low_battery(self):
        self.battery = max(0.0, self.low_threshold - 1.0)
        self._auto_return_armed = True
        self.get_logger().warn(
            f'演示模式：电量强制设为 {self.battery:.1f}%，准备自动返航')

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

    def _handle_key(self, key: str) -> bool:
        if key in ('q', 'Q', '\x03'):
            return False
        if key in ('w', 'W', 'up'):
            self._manual_override(self.linear_speed, 0.0)
        elif key in ('x', 'X', 'down'):
            self._manual_override(-self.linear_speed, 0.0)
        elif key in ('a', 'A', 'left'):
            self._manual_override(0.0, self.angular_speed)
        elif key in ('d', 'D', 'right'):
            self._manual_override(0.0, -self.angular_speed)
        elif key in ('s', 'S', ' ', 'k', 'K'):
            self._stop_motion()
        elif key in ('h', 'H'):
            self._start_return()
        elif key in ('b', 'B'):
            self._force_low_battery()
        elif key in ('+', '='):
            self.linear_speed = min(2.0, self.linear_speed + 0.05)
            self.get_logger().info(
                f'线速度调整为 {self.linear_speed:.2f} m/s')
        elif key in ('-', '_'):
            self.linear_speed = max(0.05, self.linear_speed - 0.05)
            self.get_logger().info(
                f'线速度调整为 {self.linear_speed:.2f} m/s')
        return True

    def print_usage(self):
        self.get_logger().info('差速小车电量管理与返航已启动')
        self.get_logger().info('  w/x 或 ↑/↓ : 前进/后退')
        self.get_logger().info('  a/d 或 ←/→ : 左转/右转')
        self.get_logger().info('  s 或空格   : 停止/暂停返航')
        self.get_logger().info('  h          : 手动返回充电点')
        self.get_logger().info('  b          : 演示用，强制低电量')
        self.get_logger().info('  +/-        : 调整线速度')
        self.get_logger().info('  q/Ctrl-C   : 退出')

    # ------------------------------------------------------------------
    # 发布
    # ------------------------------------------------------------------
    def _publish_cmd(self):
        msg = Twist()
        msg.linear.x = self._cmd_linear
        msg.angular.z = self._cmd_angular
        self.cmd_publisher.publish(msg)

    def _publish_battery(self, force: bool = False):
        now = time.monotonic()
        if not force and now - self._last_battery_publish < (
                1.0 / self.battery_publish_rate):
            return
        self._last_battery_publish = now

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.percentage = self.battery / 100.0
        msg.voltage = 10.0 + 0.02 * self.battery
        msg.current = -1.0 * abs(self._odom_v) if self._mode != 'charging' else 1.5
        msg.charge = 10.0 * msg.percentage
        msg.capacity = 10.0
        msg.design_capacity = 10.0
        msg.present = True
        if self._mode == 'charging':
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif self._mode == 'returning':
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = (
            BatteryState.POWER_SUPPLY_TECHNOLOGY_LION)
        msg.location = 'main'
        msg.serial_number = 'sim-battery'
        self.battery_publisher.publish(msg)

        if abs(self.battery - self._last_battery_log) >= 5.0:
            self._last_battery_log = self.battery
            self.get_logger().info(
                f'电量 {self.battery:.1f}%  模式 {self._mode}')

    def _publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        home = Marker()
        home.header.frame_id = 'odom'
        home.header.stamp = stamp
        home.ns = 'battery_return'
        home.id = 0
        home.type = Marker.CYLINDER
        home.action = Marker.ADD
        home.pose.position.x = self.home.x
        home.pose.position.y = self.home.y
        home.pose.position.z = 0.05
        home.pose.orientation.w = 1.0
        home.scale.x = 0.3
        home.scale.y = 0.3
        home.scale.z = 0.1
        home.color.r = 0.0
        home.color.g = 1.0
        home.color.b = 0.2
        home.color.a = 0.9
        markers.markers.append(home)

        if self._mode == 'returning':
            target = Marker()
            target.header.frame_id = 'odom'
            target.header.stamp = stamp
            target.ns = 'battery_return'
            target.id = 1
            target.type = Marker.ARROW
            target.action = Marker.ADD
            target.pose.position.x = self.home.x
            target.pose.position.y = self.home.y
            target.pose.position.z = 0.2
            target.pose.orientation.w = 1.0
            target.scale.x = 0.4
            target.scale.y = 0.08
            target.scale.z = 0.08
            target.color.r = 1.0
            target.color.g = 0.1
            target.color.b = 0.1
            target.color.a = 1.0
            markers.markers.append(target)

        self.marker_publisher.publish(markers)

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
                self._update_battery(dt)
                if self._mode == 'returning':
                    self._update_return()
                self._publish_cmd()
                self._publish_battery()
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
        node = BatteryTeleop()
        node.run()
    except RuntimeError as exc:
        print(f'[battery_teleop] 错误: {exc}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
