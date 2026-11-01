"""Publishes a static occupancy grid of a small indoor scene.

The grid is the input for the path planner. It is published with a
transient-local QoS so a planner started later still receives it.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid

FREE = 0
OCCUPIED = 100


class MapPublisher(Node):

    def __init__(self):
        super().__init__('map_publisher')

        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('width', 100)
        self.declare_parameter('height', 100)

        self.resolution = float(self.get_parameter('resolution').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)

        if self.width <= 0 or self.height <= 0 or self.resolution <= 0.0:
            self.get_logger().fatal('resolution, width and height must be positive')
            raise ValueError('invalid map parameters')

        self.grid = self.build_grid()

        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(OccupancyGrid, '/map', qos)
        self.create_timer(1.0, self.publish_map)

        occupied = sum(1 for v in self.grid if v == OCCUPIED)
        self.get_logger().info(
            'Map ready: %d x %d cells at %.2f m/cell, %d occupied'
            % (self.width, self.height, self.resolution, occupied))

    def build_grid(self):
        w = self.width
        h = self.height
        grid = [FREE] * (w * h)

        def fill(x0, x1, y0, y1):
            for y in range(max(0, y0), min(h, y1)):
                base = y * w
                for x in range(max(0, x0), min(w, x1)):
                    grid[base + x] = OCCUPIED

        # outer walls, two cells thick
        fill(0, w, 0, 2)
        fill(0, w, h - 2, h)
        fill(0, 2, 0, h)
        fill(w - 2, w, 0, h)

        # two obstacles the straight line has to go around
        fill(28, 46, 18, 34)
        fill(54, 72, 58, 72)

        # a wall across the middle with a single gap
        fill(10, 36, 48, 52)
        fill(44, 90, 48, 52)

        return grid

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
