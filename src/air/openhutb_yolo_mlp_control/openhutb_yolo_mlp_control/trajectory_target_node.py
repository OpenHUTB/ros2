import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


def make_figure8(
    x0,
    y0,
    z0,
    scale=10.0,
    points=80,
):
    trajectory = []

    for i in range(
        points + 1
    ):
        t = (
            2.0
            * math.pi
            * i
            / points
        )

        trajectory.append((
            x0
            + scale
            * math.sin(t),

            y0
            + scale
            * math.sin(t)
            * math.cos(t),

            z0,
        ))

    return trajectory


class TrajectoryTargetNode(Node):
    """在无人机当前高度发布八字形轨迹目标点。"""

    def __init__(self):
        super().__init__(
            'trajectory_target'
        )

        self.declare_parameter(
            'figure8_scale',
            10.0,
        )
        self.declare_parameter(
            'figure8_points',
            80,
        )
        self.declare_parameter(
            'waypoint_tolerance',
            0.8,
        )
        self.declare_parameter(
            'max_waypoint_time',
            15.0,
        )

        self.declare_parameter(
            'odom_topic',
            '/openhutb/odom',
        )
        self.declare_parameter(
            'target_topic',
            '/openhutb/target',
        )
        self.declare_parameter(
            'done_topic',
            '/openhutb/trajectory_done',
        )

        self.tolerance = float(
            self.get_parameter(
                'waypoint_tolerance'
            ).value
        )
        self.max_waypoint_time = float(
            self.get_parameter(
                'max_waypoint_time'
            ).value
        )

        self.target_pub = (
            self.create_publisher(
                PointStamped,
                str(
                    self.get_parameter(
                        'target_topic'
                    ).value
                ),
                10,
            )
        )

        self.done_pub = (
            self.create_publisher(
                Bool,
                str(
                    self.get_parameter(
                        'done_topic'
                    ).value
                ),
                10,
            )
        )

        self.odom_sub = (
            self.create_subscription(
                Odometry,
                str(
                    self.get_parameter(
                        'odom_topic'
                    ).value
                ),
                self.odom_callback,
                qos_profile_sensor_data,
            )
        )

        self.publish_timer = (
            self.create_timer(
                0.1,
                self.publish_target,
            )
        )

        self.current_position = None
        self.waypoints = None
        self.index = 0
        self.waypoint_start = None
        self.done = False

        self.get_logger().info(
            'Figure-eight target node ready; '
            'waiting for /openhutb/odom.'
        )

    def initialize_trajectory(
        self,
        x0,
        y0,
        current_z,
    ):
        scale = float(
            self.get_parameter(
                'figure8_scale'
            ).value
        )
        points = int(
            self.get_parameter(
                'figure8_points'
            ).value
        )

        # 关键安全设计：保留 OpenHUTB 报告的当前 Z，不强制设为 -8 m。
        # 之前运行时 z 约为 +49 m，强制设为 -8 m 会产生约 57 m 的垂直误差，
        # 从而导致 MLP 指令饱和。
        z0 = float(
            current_z
        )

        self.waypoints = (
            make_figure8(
                x0,
                y0,
                z0,
                scale,
                points,
            )
        )

        self.index = 0
        self.waypoint_start = (
            time.monotonic()
        )

        self.get_logger().info(
            'Figure-eight initialized: '
            f'{len(self.waypoints)} waypoints, '
            f'origin=({x0:.2f}, {y0:.2f}), '
            f'z={z0:.2f}, '
            f'scale={scale:.2f}'
        )

    def odom_callback(
        self,
        msg: Odometry,
    ):
        p = msg.pose.pose.position

        self.current_position = (
            float(p.x),
            float(p.y),
            float(p.z),
        )

        if self.waypoints is None:
            self.initialize_trajectory(
                *self.current_position
            )

        if self.done:
            return

        tx, ty, tz = (
            self.waypoints[
                self.index
            ]
        )
        x, y, z = (
            self.current_position
        )

        # 航点完成条件使用水平距离。垂直控制会独立修正高度漂移，
        # 而姿态后备控制仍可能报告少量高度偏差；若把 Z 误差计入航点切换，
        # 无人机即使已在 XY 平面到达航点，也可能一直等待到超时。
        distance = math.hypot(
            tx - x,
            ty - y,
        )

        timed_out = (
            time.monotonic()
            - self.waypoint_start
            > self.max_waypoint_time
        )

        if (
            distance
            < self.tolerance
            or timed_out
        ):
            if timed_out:
                self.get_logger().warning(
                    f'Waypoint '
                    f'{self.index + 1} '
                    f'timed out at '
                    f'{distance:.2f} m; '
                    'advancing.'
                )

            self.index += 1

            if self.index >= len(
                self.waypoints
            ):
                self.done = True

                done_msg = Bool()
                done_msg.data = True
                self.done_pub.publish(
                    done_msg
                )

                self.get_logger().info(
                    'Figure-eight trajectory '
                    'completed.'
                )
                return

            self.waypoint_start = (
                time.monotonic()
            )

            self.get_logger().info(
                f'Waypoint -> '
                f'{self.index + 1}/'
                f'{len(self.waypoints)}'
            )

    def publish_target(self):
        if (
            self.waypoints is None
            or self.done
        ):
            return

        tx, ty, tz = (
            self.waypoints[
                self.index
            ]
        )

        msg = PointStamped()
        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )
        msg.header.frame_id = (
            'airsim_ned'
        )

        msg.point.x = float(tx)
        msg.point.y = float(ty)
        msg.point.z = float(tz)

        self.target_pub.publish(
            msg
        )


def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryTargetNode()

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
