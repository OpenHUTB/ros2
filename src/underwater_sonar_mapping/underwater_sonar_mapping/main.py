"""ROS node that turns odometry and simulated sonar returns into a 2D map."""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from underwater_sonar_mapping.sonar_core import (
    DEFAULT_INITIAL_POSE,
    DEFAULT_MAP_ORIGIN,
    DEFAULT_OBSTACLES,
    integrate_scan,
    raycast_scan,
    validate_obstacles,
)


class UnderwaterSonarMapping(Node):
    def __init__(self):
        super().__init__("underwater_sonar_mapping")
        self.declare_parameter("map_width", 20.0)
        self.declare_parameter("map_height", 20.0)
        self.declare_parameter("resolution", 0.25)
        self.declare_parameter("beam_count", 90)
        self.declare_parameter("field_of_view", math.tau)
        self.declare_parameter("max_range", 10.0)
        self.declare_parameter("scan_rate", 5.0)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("initial_pose", list(DEFAULT_INITIAL_POSE))
        self.declare_parameter(
            "obstacles", [value for obstacle in DEFAULT_OBSTACLES for value in obstacle]
        )

        self.resolution = float(self.get_parameter("resolution").value)
        map_width = self._positive_float("map_width")
        map_height = self._positive_float("map_height")
        self.beam_count = int(self.get_parameter("beam_count").value)
        self.field_of_view = float(self.get_parameter("field_of_view").value)
        self.max_range = self._positive_float("max_range")
        scan_rate = self._positive_float("scan_rate")
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError("resolution must be finite and positive")
        if self.beam_count < 1:
            raise ValueError("beam_count must be positive")
        if not math.isfinite(self.field_of_view) or not 0 < self.field_of_view <= math.tau:
            raise ValueError("field_of_view must be in (0, 2*pi]")

        width = math.ceil(map_width / self.resolution)
        height = math.ceil(map_height / self.resolution)
        if width * height > 4_000_000:
            raise ValueError("map dimensions exceed 4 million cells")
        self.map_origin = DEFAULT_MAP_ORIGIN
        self.grid = np.full((height, width), -1, dtype=np.int8)

        raw_obstacles = list(self.get_parameter("obstacles").value)
        if len(raw_obstacles) % 3:
            raise ValueError("obstacles must be a flat list of x, y, radius triples")
        self.obstacles = validate_obstacles(
            [raw_obstacles[index : index + 3] for index in range(0, len(raw_obstacles), 3)]
        )

        initial_pose = list(self.get_parameter("initial_pose").value)
        if len(initial_pose) != 3 or not all(math.isfinite(float(value)) for value in initial_pose):
            raise ValueError("initial_pose must contain finite x, y, yaw values")
        self.pose = tuple(float(value) for value in initial_pose)
        self.odom_topic = str(self.get_parameter("odom_topic").value)

        self.map_publisher = self.create_publisher(OccupancyGrid, "sonar/map", 10)
        self.echo_publisher = self.create_publisher(PointCloud2, "sonar/echoes", 10)
        self.odom_subscription = self.create_subscription(Odometry, self.odom_topic, self._on_odometry, 10)
        self.timer = self.create_timer(1.0 / scan_rate, self._publish_scan)

    def _positive_float(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def _on_odometry(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.pose = (position.x, position.y, yaw)

    def _publish_scan(self):
        x, y, yaw = self.pose
        scan = raycast_scan(
            x,
            y,
            yaw,
            self.obstacles,
            beam_count=self.beam_count,
            field_of_view=self.field_of_view,
            max_range=self.max_range,
        )
        self.grid = integrate_scan(self.grid, self.map_origin, self.resolution, x, y, scan)

        stamp = self.get_clock().now().to_msg()
        occupancy = OccupancyGrid()
        occupancy.header.stamp = stamp
        occupancy.header.frame_id = "map"
        occupancy.info.resolution = self.resolution
        occupancy.info.width = self.grid.shape[1]
        occupancy.info.height = self.grid.shape[0]
        occupancy.info.origin.position.x = self.map_origin[0]
        occupancy.info.origin.position.y = self.map_origin[1]
        occupancy.info.origin.orientation.w = 1.0
        occupancy.data = self.grid.reshape(-1).tolist()
        self.map_publisher.publish(occupancy)

        header = Header()
        header.stamp = stamp
        header.frame_id = "map"
        echoes = [
            (x + math.cos(angle) * distance, y + math.sin(angle) * distance, 0.0)
            for angle, distance, hit in scan
            if hit
        ]
        self.echo_publisher.publish(point_cloud2.create_cloud_xyz32(header, echoes))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UnderwaterSonarMapping()
        rclpy.spin(node)
    except (ValueError, TypeError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"Invalid underwater sonar mapping configuration: {error}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
