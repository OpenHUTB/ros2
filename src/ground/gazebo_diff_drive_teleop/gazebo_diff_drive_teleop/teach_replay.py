#!/usr/bin/env python3
"""Gazebo 差速小车示教录制与路径回放节点。

键盘手动驾驶时记录 /odom 轨迹，按 p 后由纯追踪控制器沿记录轨迹自动
行驶回另一端。支持循环往返和手动接管，适合演示“示教一次，重复执行”。
"""

import math
import os
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

ARROW_KEYS = {
    '\x1b[A': 'up',
    '\x1b[B': 'down',
    '\x1b[C': 'right',
    '\x1b[D': 'left',
}


@dataclass
class Pose2D:
    """二维平面位姿。"""

    x: float
    y: float
    yaw: float


def normalize_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi]。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def distance(a: Pose2D, b: Pose2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


class TeachReplay(Node):
    """键盘示教 + 里程计轨迹回放节点。"""

    def __init__(self):
        super().__init__('teach_replay')

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('path_topic', 'teach_replay/path')
        self.declare_parameter('marker_topic', 'teach_replay/markers')
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('record_min_distance', 0.05)
        self.declare_parameter('record_min_angle', 0.10)
        self.declare_parameter('lookahead_distance', 0.25)
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('replay_linear_gain', 1.2)
        self.declare_parameter('replay_angular_gain', 2.0)
        self.declare_parameter('max_path_points', 5000)
        self.declare_parameter('loop_replay', False)

        self.linear_speed = max(
            0.05, float(self.get_parameter('linear_speed').value))
        self.angular_speed = max(
            0.1, float(self.get_parameter('angular_speed').value))
        self.publish_rate = max(
            1.0, float(self.get_parameter('publish_rate').value))
        self.record_min_distance = max(
            0.01, float(self.get_parameter('record_min_distance').value))
        self.record_min_angle = max(
            0.01, float(self.get_parameter('record_min_angle').value))
        self.lookahead_distance = max(
            0.05, float(self.get_parameter('lookahead_distance').value))
        self.goal_tolerance = max(
            0.02, float(self.get_parameter('goal_tolerance').value))
        self.replay_linear_gain = max(
            0.1, float(self.get_parameter('replay_linear_gain').value))
        self.replay_angular_gain = max(
            0.1, float(self.get_parameter('replay_angular_gain').value))
        self.max_path_points = max(
            10, int(self.get_parameter('max_path_points').value))
        self.loop_replay = bool(self.get_parameter('loop_replay').value)

        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)

        self.cmd_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.path_publisher = self.create_publisher(Path, path_topic, 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, marker_topic, 10)

        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Odometry, odom_topic, self._on_odom, odom_qos)

        self._pose: Optional[Pose2D] = None
        self._odom_frame = 'odom'
        self._points: List[Pose2D] = []
        self._mode = 'teleop'  # teleop / record / replay
        self._running = True
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._replay_direction = 1
        self._replay_target: Optional[Pose2D] = None
        self._path_dirty = False

        self._stdin_fd = sys.stdin.fileno()
        self._original_term_settings = None
        self._key_buffer = b''

        if not sys.stdin.isatty():
            raise RuntimeError(
                '标准输入不是交互式终端，无法读取按键。'
                '请在终端中直接运行本节点。'
            )

    # ------------------------------------------------------------------
    # ROS 回调
    # ------------------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        siny = 2.0 * (orientation.w * orientation.z
                      + orientation.x * orientation.y)
        cosy = 1.0 - 2.0 * (orientation.y * orientation.y
                           + orientation.z * orientation.z)
        yaw = normalize_angle(math.atan2(siny, cosy))
        self._pose = Pose2D(position.x, position.y, yaw)
        self._odom_frame = msg.header.frame_id or 'odom'

        if self._mode == 'record':
            self._record_pose(self._pose)

    # ------------------------------------------------------------------
    # 轨迹录制
    # ------------------------------------------------------------------
    def _record_pose(self, pose: Pose2D):
        if not self._points:
            self._points.append(pose)
            self._path_dirty = True
            return

        last = self._points[-1]
        moved = distance(pose, last) >= self.record_min_distance
        turned = abs(normalize_angle(pose.yaw - last.yaw)) >= self.record_min_angle
        if not (moved or turned):
            return

        if len(self._points) >= self.max_path_points:
            self._finish_recording('达到最大采样点数，自动结束录制')
            return

        self._points.append(pose)
        self._path_dirty = True

    def _start_recording(self):
        self._points.clear()
        self._mode = 'record'
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        if self._pose is not None:
            self._record_pose(self._pose)
        self._path_dirty = True
        self.get_logger().info('开始录制：用 w/x/a/d 或方向键驾驶，按 r 结束')

    def _finish_recording(self, reason: str = '录制结束'):
        self._mode = 'teleop'
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._path_dirty = True
        self.get_logger().info(
            f'{reason}，共记录 {len(self._points)} 个路径点')

    def _toggle_recording(self):
        if self._mode == 'record':
            self._finish_recording()
        else:
            self._start_recording()

    def _clear_path(self):
        self._points.clear()
        self._mode = 'teleop'
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._replay_target = None
        self._path_dirty = True
        self.get_logger().info('已清空示教轨迹')

    # ------------------------------------------------------------------
    # 路径回放
    # ------------------------------------------------------------------
    def _start_replay(self):
        if len(self._points) < 2:
            self.get_logger().warn('路径点不足，请先录制一段轨迹')
            return
        if self._pose is None:
            self.get_logger().warn('尚未收到 /odom，无法回放')
            return

        dist_to_start = distance(self._pose, self._points[0])
        dist_to_end = distance(self._pose, self._points[-1])
        # 从离当前位置较近的端点出发，开向另一端
        self._replay_direction = -1 if dist_to_end < dist_to_start else 1
        self._mode = 'replay'
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        direction_name = '反向' if self._replay_direction < 0 else '正向'
        self.get_logger().info(
            f'开始{direction_name}回放，共 {len(self._points)} 个路径点')

    def _toggle_replay(self):
        if self._mode == 'replay':
            self._stop_motion('回放已暂停')
        elif self._mode == 'record':
            self._finish_recording('录制结束，准备回放')
            self._start_replay()
        else:
            self._start_replay()

    def _stop_motion(self, reason: str = '已停止'):
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        if self._mode == 'replay':
            self._mode = 'teleop'
            self._replay_target = None
            self.get_logger().info(reason)

    def _update_replay(self):
        if self._mode != 'replay' or self._pose is None:
            return
        if len(self._points) < 2:
            self._stop_motion('路径点不足，停止回放')
            return

        pose = self._pose
        goal_index = 0 if self._replay_direction < 0 else len(self._points) - 1
        goal = self._points[goal_index]

        if distance(pose, goal) <= self.goal_tolerance:
            self._cmd_linear = 0.0
            self._cmd_angular = 0.0
            if self.loop_replay:
                self._replay_direction *= -1
                self._replay_target = None
                self.get_logger().info('到达端点，反向继续循环回放')
            else:
                self._mode = 'teleop'
                self._replay_target = None
                self.get_logger().info('回放完成，已到达目标端点')
            return

        nearest_index = min(
            range(len(self._points)),
            key=lambda i: distance(pose, self._points[i]),
        )

        target_index = nearest_index
        if self._replay_direction > 0:
            for i in range(nearest_index, len(self._points)):
                if distance(pose, self._points[i]) >= self.lookahead_distance:
                    target_index = i
                    break
            else:
                target_index = len(self._points) - 1
        else:
            for i in range(nearest_index, -1, -1):
                if distance(pose, self._points[i]) >= self.lookahead_distance:
                    target_index = i
                    break
            else:
                target_index = 0

        target = self._points[target_index]
        self._replay_target = target

        dx = target.x - pose.x
        dy = target.y - pose.y
        local_x = math.cos(pose.yaw) * dx + math.sin(pose.yaw) * dy
        local_y = -math.sin(pose.yaw) * dx + math.cos(pose.yaw) * dy
        target_distance = math.hypot(local_x, local_y)
        heading_error = math.atan2(local_y, local_x)

        if abs(heading_error) > 1.0:
            linear = 0.0
            angular = math.copysign(self.angular_speed, heading_error)
        else:
            linear = min(self.linear_speed,
                         self.replay_linear_gain * target_distance)
            goal_distance = distance(pose, goal)
            if goal_distance < 0.35:
                slow = 0.6 * self.linear_speed * goal_distance / 0.35
                linear = min(linear, max(0.04, slow))
            if abs(heading_error) > 0.35:
                linear *= 0.5
            angular = max(-self.angular_speed,
                          min(self.angular_speed,
                              self.replay_angular_gain * heading_error))

        self._cmd_linear = linear
        self._cmd_angular = angular

    # ------------------------------------------------------------------
    # 键盘
    # ------------------------------------------------------------------
    def _enable_cbreak(self):
        self._original_term_settings = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)

    def _restore_terminal(self):
        if self._original_term_settings is not None:
            termios.tcsetattr(
                self._stdin_fd,
                termios.TCSADRAIN,
                self._original_term_settings,
            )
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

    def _set_manual_velocity(self, linear: float, angular: float):
        if self._mode == 'replay':
            self._mode = 'teleop'
            self._replay_target = None
            self.get_logger().info('检测到手动按键，退出自动回放')
        self._cmd_linear = linear
        self._cmd_angular = angular

    def _adjust_speed(self, delta: float):
        old_speed = self.linear_speed
        self.linear_speed = max(0.05, min(2.0, self.linear_speed + delta))
        if self.linear_speed == old_speed:
            return
        if self._cmd_linear > 0.0:
            self._cmd_linear = self.linear_speed
        elif self._cmd_linear < 0.0:
            self._cmd_linear = -self.linear_speed
        self.get_logger().info(f'线速度调整为 {self.linear_speed:.2f} m/s')

    def _handle_key(self, key: str) -> bool:
        if key in ('q', 'Q', '\x03'):
            return False
        if key in ('r', 'R'):
            self._toggle_recording()
        elif key in ('p', 'P'):
            self._toggle_replay()
        elif key in ('c', 'C'):
            self._clear_path()
        elif key in ('l', 'L'):
            self.loop_replay = not self.loop_replay
            self.get_logger().info(
                f'循环回放: {"开" if self.loop_replay else "关"}')
        elif key in ('s', 'S', ' ', 'k', 'K'):
            self._stop_motion()
        elif key in ('+', '='):
            self._adjust_speed(0.05)
        elif key in ('-', '_'):
            self._adjust_speed(-0.05)
        elif key in ('w', 'W', 'up'):
            self._set_manual_velocity(self.linear_speed, 0.0)
        elif key in ('x', 'X', 'down'):
            self._set_manual_velocity(-self.linear_speed, 0.0)
        elif key in ('a', 'A', 'left'):
            self._set_manual_velocity(0.0, self.angular_speed)
        elif key in ('d', 'D', 'right'):
            self._set_manual_velocity(0.0, -self.angular_speed)
        elif key in ('h', 'H', '?'):
            self.print_usage()
        return True

    def print_usage(self):
        self.get_logger().info('差速小车示教/回放已启动')
        self.get_logger().info('  w/x 或 ↑/↓ : 前进/后退')
        self.get_logger().info('  a/d 或 ←/→ : 左转/右转')
        self.get_logger().info('  s 或空格   : 停止/暂停回放')
        self.get_logger().info('  r          : 开始/结束录制（重新录制会清空旧轨迹）')
        self.get_logger().info('  p          : 开始/暂停回放')
        self.get_logger().info('  l          : 循环回放开关')
        self.get_logger().info('  c          : 清空轨迹')
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

    def _make_pose_stamped(self, pose: Pose2D, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self._odom_frame
        msg.header.stamp = stamp
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        msg.pose.orientation.z = math.sin(pose.yaw * 0.5)
        msg.pose.orientation.w = math.cos(pose.yaw * 0.5)
        return msg

    def _publish_path(self):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = self._odom_frame
        path.header.stamp = stamp
        path.poses = [
            self._make_pose_stamped(point, stamp) for point in self._points
        ]
        self.path_publisher.publish(path)
        self._path_dirty = False

    def _make_sphere_marker(self, marker_id: int, pose: Pose2D,
                            color, scale: float, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._odom_frame
        marker.header.stamp = stamp
        marker.ns = 'teach_replay'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pose.x
        marker.pose.position.y = pose.y
        marker.pose.position.z = 0.15
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        return marker

    def _publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        if not self._points:
            delete_all = Marker()
            delete_all.action = Marker.DELETEALL
            markers.markers.append(delete_all)
        else:
            markers.markers.append(self._make_sphere_marker(
                0, self._points[0], (0.0, 1.0, 0.0), 0.08, stamp))
            markers.markers.append(self._make_sphere_marker(
                1, self._points[-1], (0.0, 0.4, 1.0), 0.08, stamp))

        if self._replay_target is not None and self._mode == 'replay':
            markers.markers.append(self._make_sphere_marker(
                2, self._replay_target, (1.0, 0.0, 0.0), 0.10, stamp))
        else:
            delete_target = Marker()
            delete_target.header.frame_id = self._odom_frame
            delete_target.header.stamp = stamp
            delete_target.ns = 'teach_replay'
            delete_target.id = 2
            delete_target.action = Marker.DELETE
            markers.markers.append(delete_target)

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

        try:
            while rclpy.ok() and self._running:
                loop_start = time.monotonic()
                for key in self._read_keys(period):
                    if not self._handle_key(key):
                        self._running = False
                        break
                rclpy.spin_once(self, timeout_sec=0.0)
                if self._mode == 'replay':
                    self._update_replay()
                self._publish_cmd()
                if self._path_dirty:
                    self._publish_path()
                self._publish_markers()
                elapsed = time.monotonic() - loop_start
                if elapsed < period:
                    time.sleep(period - elapsed)
        except (KeyboardInterrupt, EOFError):
            pass
        finally:
            self._restore_terminal()
            self._mode = 'teleop'
            self._cmd_linear = 0.0
            self._cmd_angular = 0.0
            self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TeachReplay()
        node.run()
    except RuntimeError as exc:
        print(f'[teach_replay] 错误: {exc}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
