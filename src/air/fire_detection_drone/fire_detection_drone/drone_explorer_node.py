"""ROS 2 exploration node driven by fire alerts and a deterministic FSM."""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String

from .explorer_core import (
    DetectionInfo,
    ExplorerConfig,
    ExplorerCore,
    Pose2D,
)
from .pose_utils import (
    enu_twist_to_ned,
    ned_pose_to_enu,
    normalize_angle,
    yaw_from_quaternion,
)


class DroneExplorerNode(Node):

    def __init__(self) -> None:
        super().__init__('drone_explorer')

        self.pose_topic = str(self.declare_parameter(
            'pose_topic', '/odom'
        ).value)
        self.pose_message_type = str(self.declare_parameter(
            'pose_message_type', 'odometry'
        ).value).strip().lower()
        self.alert_topic = str(self.declare_parameter(
            'alert_topic', '/fire_detection/alert'
        ).value)
        self.bbox_topic = str(self.declare_parameter(
            'bbox_topic', '/fire_detection/bbox'
        ).value)
        self.abort_topic = str(self.declare_parameter(
            'abort_topic', '/fire_detection/abort'
        ).value)
        self.cmd_vel_topic = str(self.declare_parameter(
            'cmd_vel_topic', '/cmd_vel'
        ).value)
        self.land_topic = str(self.declare_parameter(
            'land_topic', '/drone/land'
        ).value)

        self.require_odometry = bool(self.declare_parameter(
            'require_odometry', True
        ).value)
        self.use_initial_pose_as_home = bool(self.declare_parameter(
            'use_initial_pose_as_home', True
        ).value)
        self.ned_input = bool(self.declare_parameter(
            'ned_input', False
        ).value)
        self.ned_output = bool(self.declare_parameter(
            'ned_output', False
        ).value)
        self.pose_timeout_sec = max(0.1, float(self.declare_parameter(
            'pose_timeout_sec', 1.5
        ).value))
        self.alert_timeout_sec = max(0.2, float(self.declare_parameter(
            'alert_timeout_sec', 3.0
        ).value))
        self.bbox_timeout_sec = max(0.1, float(self.declare_parameter(
            'bbox_timeout_sec', 1.0
        ).value))
        self.max_linear_speed = max(0.1, float(self.declare_parameter(
            'max_linear_speed', 2.0
        ).value))
        self.max_vertical_speed = max(0.1, float(self.declare_parameter(
            'max_vertical_speed', 1.0
        ).value))
        self.max_angular_speed = max(0.1, float(self.declare_parameter(
            'max_angular_speed', 1.0
        ).value))

        pattern = str(self.declare_parameter(
            'exploration_pattern', 'lawnmower'
        ).value).strip().lower()
        if pattern not in ExplorerCore.supported_patterns():
            self.get_logger().warning(
                f'Unknown exploration_pattern="{pattern}", '
                'falling back to lawnmower'
            )
            pattern = 'lawnmower'

        config = ExplorerConfig(
            pattern=pattern,
            search_width=float(self.declare_parameter(
                'search_width', 12.0
            ).value),
            search_height=float(self.declare_parameter(
                'search_height', 8.0
            ).value),
            lane_spacing=float(self.declare_parameter(
                'lane_spacing', 2.5
            ).value),
            spiral_turns=float(self.declare_parameter(
                'spiral_turns', 3.0
            ).value),
            search_altitude=float(self.declare_parameter(
                'search_altitude', 3.0
            ).value),
            patrol_speed=float(self.declare_parameter(
                'patrol_speed', 0.8
            ).value),
            return_speed=float(self.declare_parameter(
                'return_speed', 1.0
            ).value),
            vertical_speed=float(self.declare_parameter(
                'vertical_speed', 0.4
            ).value),
            max_yaw_rate=float(self.declare_parameter(
                'max_yaw_rate', 0.6
            ).value),
            yaw_gain=float(self.declare_parameter(
                'yaw_gain', 1.2
            ).value),
            waypoint_tolerance=float(self.declare_parameter(
                'waypoint_tolerance', 0.8
            ).value),
            confirm_hover_sec=float(self.declare_parameter(
                'confirm_hover_sec', 2.0
            ).value),
            alert_grace_sec=float(self.declare_parameter(
                'alert_grace_sec', 3.0
            ).value),
            evidence_sec=float(self.declare_parameter(
                'evidence_sec', 12.0
            ).value),
            evidence_yaw_rate=float(self.declare_parameter(
                'evidence_yaw_rate', 0.5
            ).value),
            return_home_tolerance=float(self.declare_parameter(
                'return_home_tolerance', 0.6
            ).value),
            max_mission_sec=float(self.declare_parameter(
                'max_mission_sec', 300.0
            ).value),
            land_after_return=bool(self.declare_parameter(
                'land_after_return', True
            ).value),
            home_x=float(self.declare_parameter('home_x', 0.0).value),
            home_y=float(self.declare_parameter('home_y', 0.0).value),
            home_z=float(self.declare_parameter('home_z', 0.0).value),
        )
        self.core = ExplorerCore(config)

        control_rate = max(1.0, float(self.declare_parameter(
            'control_rate', 20.0
        ).value))
        status_rate = max(0.2, float(self.declare_parameter(
            'status_rate', 1.0
        ).value))

        self.pose = Pose2D(
            x=config.home_x,
            y=config.home_y,
            z=config.home_z,
        )
        self.alert = False
        self.abort_requested = False
        self.pose_source = 'waiting'
        self._pose_received = False
        self._home_initialized = not self.use_initial_pose_as_home
        self._last_pose_time: float | None = None
        self._last_alert_time: float | None = None
        self._last_bbox_time: float | None = None
        self._last_bbox: list[float] = []
        self._last_core_command = None
        self._last_control_time: float | None = None
        self._mission_started_at: float | None = None
        self._land_published = False
        self._warned_pose = False
        self._warned_pose_lost = False

        if not self.require_odometry and not self._home_initialized:
            self._home_initialized = True

        if self.pose_message_type in ('pose_stamped', 'pose', 'posestamped'):
            self.pose_sub = self.create_subscription(
                PoseStamped, self.pose_topic, self.pose_stamped_callback, 10
            )
            self.pose_source = 'pose_stamped'
        else:
            if self.pose_message_type != 'odometry':
                self.get_logger().warning(
                    f'Unknown pose_message_type="{self.pose_message_type}", '
                    'using nav_msgs/Odometry'
                )
            self.pose_sub = self.create_subscription(
                Odometry, self.pose_topic, self.odometry_callback, 10
            )
            self.pose_source = 'odometry'

        self.alert_sub = self.create_subscription(
            Bool, self.alert_topic, self.alert_callback, 10
        )
        self.bbox_sub = self.create_subscription(
            Float32MultiArray, self.bbox_topic, self.bbox_callback, 10
        )
        self.abort_sub = self.create_subscription(
            Bool, self.abort_topic, self.abort_callback, 10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist, self.cmd_vel_topic, 10
        )
        self.land_pub = self.create_publisher(Bool, self.land_topic, 10)
        self.status_pub = self.create_publisher(
            String, '/fire_detection/mission_status', 10
        )

        self.create_timer(1.0 / control_rate, self.control_loop)
        self.create_timer(1.0 / status_rate, self.publish_status)

        self.get_logger().info(
            f'Explorer ready: pattern={pattern}, pose={self.pose_topic} '
            f'({self.pose_source}), cmd_vel={self.cmd_vel_topic}, '
            f'require_odometry={self.require_odometry}'
        )

    def odometry_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self._apply_pose(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            yaw,
            'odometry',
        )

    def pose_stamped_callback(self, msg: PoseStamped) -> None:
        pose = msg.pose
        yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self._apply_pose(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            yaw,
            'pose_stamped',
        )

    def _apply_pose(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        source: str,
    ) -> None:
        if self.ned_input:
            x, y, z, yaw = ned_pose_to_enu(x, y, z, yaw)
        self.pose = Pose2D(x=float(x), y=float(y), z=float(z), yaw=float(yaw))
        self.pose_source = source
        self._pose_received = True
        self._last_pose_time = self._now_sec()
        self._warned_pose = False
        self._warned_pose_lost = False

        if self.use_initial_pose_as_home and not self._home_initialized:
            self.core.set_home(self.pose.x, self.pose.y, self.pose.z)
            self._home_initialized = True
            self.get_logger().info(
                f'Home position set to ({self.pose.x:.2f}, '
                f'{self.pose.y:.2f}, {self.pose.z:.2f})'
            )

    def alert_callback(self, msg: Bool) -> None:
        was_alerting = self.alert
        self.alert = bool(msg.data)
        self._last_alert_time = self._now_sec()
        if self.alert and not was_alerting:
            self.get_logger().warning('Fire alert observed by explorer')
        elif was_alerting and not self.alert:
            self.get_logger().info('Fire alert cleared')

    def bbox_callback(self, msg: Float32MultiArray) -> None:
        self._last_bbox = list(msg.data)
        self._last_bbox_time = self._now_sec()

    def abort_callback(self, msg: Bool) -> None:
        if msg.data and not self.abort_requested:
            self.get_logger().warning('Abort requested: returning home')
        self.abort_requested = bool(msg.data)

    def control_loop(self) -> None:
        now = self._now_sec()
        if not self._pose_available(now):
            self._publish_stop()
            return

        if self._mission_started_at is None:
            self._mission_started_at = now

        dt = 0.0
        if self._last_control_time is not None:
            dt = min(0.2, max(0.0, now - self._last_control_time))
        self._last_control_time = now

        if not self.require_odometry:
            self._integrate_dead_reckoning(dt)
            self.pose_source = 'dead_reckoning'

        detection = self._current_detection(now)
        alert = self.alert
        if (
            self._last_alert_time is not None
            and now - self._last_alert_time > self.alert_timeout_sec
        ):
            alert = False
            self.alert = False

        previous_state = self.core.state
        command = self.core.update(
            now=now,
            pose=self.pose,
            alert=alert,
            detection=detection,
            abort=self.abort_requested,
        )
        self._last_core_command = command
        self._publish_command(command)
        self._publish_land_if_needed(command)

        if command.state != previous_state:
            self.get_logger().info(
                f'Explorer state {previous_state.value} -> '
                f'{command.state.value}: {command.reason}'
            )

    def _pose_available(self, now: float) -> bool:
        if not self.require_odometry:
            if not self._pose_received:
                self._pose_received = True
                self._last_pose_time = now
                if not self._home_initialized:
                    self.core.set_home(
                        self.core.config.home_x,
                        self.core.config.home_y,
                        self.core.config.home_z,
                    )
                    self._home_initialized = True
            return True

        if not self._pose_received or self._last_pose_time is None:
            if not self._warned_pose:
                self.get_logger().warning(
                    f'Waiting for pose on {self.pose_topic}'
                )
                self._warned_pose = True
            return False

        if now - self._last_pose_time > self.pose_timeout_sec:
            if not self._warned_pose_lost:
                self.get_logger().error(
                    'Pose timeout: stopping until pose updates resume'
                )
                self._warned_pose_lost = True
            return False
        return True

    def _integrate_dead_reckoning(self, dt: float) -> None:
        if dt <= 0.0 or self._last_core_command is None:
            return
        command = self._last_core_command
        self.pose.yaw = normalize_angle(
            self.pose.yaw + command.angular_z * dt
        )
        world_vx, world_vy = _body_to_world(
            command.linear_x, command.linear_y, self.pose.yaw
        )
        self.pose.x += world_vx * dt
        self.pose.y += world_vy * dt
        self.pose.z += command.linear_z * dt

    def _current_detection(self, now: float) -> DetectionInfo:
        if (
            self._last_bbox
            and len(self._last_bbox) >= 8
            and self._last_bbox_time is not None
            and now - self._last_bbox_time <= self.bbox_timeout_sec
        ):
            return DetectionInfo(
                center_x=float(self._last_bbox[5]),
                center_y=float(self._last_bbox[6]),
                confidence=float(self._last_bbox[4]),
                visible=True,
            )
        return DetectionInfo()

    def _publish_command(self, command) -> None:
        linear_x = _clamp(
            command.linear_x, -self.max_linear_speed, self.max_linear_speed
        )
        linear_y = _clamp(
            command.linear_y, -self.max_linear_speed, self.max_linear_speed
        )
        linear_z = _clamp(
            command.linear_z,
            -self.max_vertical_speed,
            self.max_vertical_speed,
        )
        angular_z = _clamp(
            command.angular_z,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        if self.ned_output:
            linear_x, linear_y, linear_z, angular_z = enu_twist_to_ned(
                linear_x, linear_y, linear_z, angular_z
            )

        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.linear.z = linear_z
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def _publish_stop(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def _publish_land_if_needed(self, command) -> None:
        if command.land and not self._land_published:
            self.land_pub.publish(Bool(data=True))
            self._land_published = True
            self.get_logger().warning(
                f'Landing requested on {self.land_topic}'
            )

    def publish_status(self) -> None:
        command = self._last_core_command
        now = self._now_sec()
        status = {
            'state': (
                command.state.value
                if command is not None
                else self.core.state.value
            ),
            'alert': self.alert,
            'abort': self.abort_requested,
            'pattern': self.core.config.pattern,
            'waypoint_index': (
                command.waypoint_index if command is not None else 0
            ),
            'waypoint_count': len(self.core.waypoints),
            'pose': {
                'x': round(self.pose.x, 3),
                'y': round(self.pose.y, 3),
                'z': round(self.pose.z, 3),
                'yaw': round(self.pose.yaw, 3),
            },
            'home': {
                'x': round(self.core.config.home_x, 3),
                'y': round(self.core.config.home_y, 3),
                'z': round(self.core.config.home_z, 3),
            },
            'pose_source': self.pose_source,
            'mission_elapsed_sec': (
                round(now - self._mission_started_at, 1)
                if self._mission_started_at is not None
                else 0.0
            ),
            'reason': command.reason if command is not None else 'waiting',
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def destroy_node(self) -> None:
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception:
            pass
        super().destroy_node()


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def _body_to_world(
    body_vx: float, body_vy: float, yaw: float
) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    world_vx = body_vx * cos_yaw - body_vy * sin_yaw
    world_vy = body_vx * sin_yaw + body_vy * cos_yaw
    return world_vx, world_vy


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DroneExplorerNode()
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
