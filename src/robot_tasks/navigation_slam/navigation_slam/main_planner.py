"""Global path planner on a 2D occupancy grid.

Subscribes to /map and /goal_pose, runs A* and publishes the result as
a nav_msgs/Path on /plan.
"""

import heapq
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

OCCUPIED = 100

# eight-connected, so paths do not look like stairs
NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1),
              (0, -1), (0, 1),
              (1, -1), (1, 0), (1, 1)]


class PathPlanner(Node):

    def __init__(self):
        super().__init__('path_planner')

        self.declare_parameter('start_x', 0.5)
        self.declare_parameter('start_y', 0.5)

        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)

        self.grid = None
        self.width = 0
        self.height = 0
        self.resolution = 0.1

        map_qos = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)

        self.publisher = self.create_publisher(Path, '/plan', 10)
        self.get_logger().info(
            'Planner ready, start at (%.2f, %.2f)' % (self.start_x, self.start_y))

    # ---------- callbacks ----------

    def on_map(self, msg):
        first_time = self.grid is None
        self.grid = list(msg.data)
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        if first_time:
            occupied = sum(1 for v in self.grid if v == OCCUPIED)
            self.get_logger().info(
                'Map received: %d x %d, %d occupied cells'
                % (self.width, self.height, occupied))

    def on_goal(self, msg):
        if self.grid is None:
            self.get_logger().warn('goal received before the map, ignoring')
            return

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        start = self.to_cell(self.start_x, self.start_y)
        goal = self.to_cell(goal_x, goal_y)

        if not self.is_free(goal):
            self.get_logger().warn(
                'goal (%.2f, %.2f) is outside the map or inside an obstacle'
                % (goal_x, goal_y))
            return

        cells = self.astar(start, goal)
        if cells is None:
            self.get_logger().warn(
                'no path from (%.2f, %.2f) to (%.2f, %.2f)'
                % (self.start_x, self.start_y, goal_x, goal_y))
            return

        path = self.to_path(cells, msg.header.frame_id or 'map')
        self.publisher.publish(path)

        length = 0.0
        for a, b in zip(cells, cells[1:]):
            length += math.hypot(b % self.width - a % self.width,
                                 b // self.width - a // self.width)
        length *= self.resolution

        self.get_logger().info(
            'path to (%.2f, %.2f): %d cells, %.2f m'
            % (goal_x, goal_y, len(cells), length))

    # ---------- helpers ----------

    def to_cell(self, x, y):
        cx = int(x / self.resolution)
        cy = int(y / self.resolution)
        cx = max(0, min(self.width - 1, cx))
        cy = max(0, min(self.height - 1, cy))
        return cy * self.width + cx

    def is_free(self, index):
        return 0 <= index < len(self.grid) and self.grid[index] != OCCUPIED

    def astar(self, start, goal):
        if not self.is_free(start) or not self.is_free(goal):
            return None
        if start == goal:
            return [start]

        w = self.width
        gx, gy = goal % w, goal // w

        open_heap = [(0.0, start)]
        best = {start: 0.0}
        came_from = {}
        closed = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                return self.rebuild(came_from, current)
            if current in closed:
                continue
            closed.add(current)

            cx, cy = current % w, current // w
            for dx, dy in NEIGHBOURS:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < w and 0 <= ny < self.height):
                    continue
                nxt = ny * w + nx
                if self.grid[nxt] == OCCUPIED:
                    continue
                step = math.hypot(dx, dy)
                cand = best[current] + step
                if cand < best.get(nxt, float('inf')):
                    best[nxt] = cand
                    came_from[nxt] = current
                    heapq.heappush(
                        open_heap, (cand + math.hypot(gx - nx, gy - ny), nxt))
        return None

    @staticmethod
    def rebuild(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def to_path(self, cells, frame_id):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id

        for index in cells:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = (index % self.width + 0.5) * self.resolution
            pose.pose.position.y = (index // self.width + 0.5) * self.resolution
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
