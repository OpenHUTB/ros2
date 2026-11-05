# Copyright 2026 OpenHUTB
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See http://www.apache.org/licenses/LICENSE-2.0 for the full text.
"""A* 全局路径规划节点。

订阅 ``/map``（占据栅格）与 ``/goal_pose``（目标点），在栅格上做 8 邻域 A*，
对占据栅格按 ``inflation_radius`` 膨胀出安全边界，结果以 ``nav_msgs/Path``
发布到 ``/plan`` 供控制节点跟踪。
"""

import heapq
import math
import time

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from uav_slam_nav.grid_utils import grid_to_world, world_to_grid


class Planner(Node):
    """占据栅格上的 A* 规划器。"""

    def __init__(self):
        super().__init__('planner', namespace='uav')

        self.declare_parameter('inflation_radius', 0.6)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('allow_unknown', True)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('plan_period', 0.5)
        self.declare_parameter('max_expansions', 200000)

        self._inflation = float(self.get_parameter('inflation_radius').value)
        self._allow_diagonal = bool(self.get_parameter('allow_diagonal').value)
        self._allow_unknown = bool(self.get_parameter('allow_unknown').value)
        self._goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self._plan_period = max(0.1, float(self.get_parameter('plan_period').value))
        self._max_expansions = max(1000, int(self.get_parameter('max_expansions').value))

        self._width = 0
        self._height = 0
        self._resolution = 0.25
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._blocked = None
        self._have_map = False
        self._start = None
        self._goal = None
        self._pending = False

        map_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_pub = self.create_publisher(Path, '/plan', 5)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(PoseStamped, 'pose', self._on_pose, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 10)

        self.create_timer(1.0 / self._plan_period, self._try_plan)
        self.get_logger().info(
            f'规划节点启动：膨胀半径 {self._inflation} m，'
            f'{"允许" if self._allow_diagonal else "禁止"}对角移动')

    # ------------------------------------------------------------------ 订阅
    def _on_map(self, msg):
        """缓存地图并预先膨胀障碍，避免每次规划都重算。"""
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0:
            return
        data = np.asarray(msg.data, dtype=np.int8).reshape(height, width)
        if self._allow_unknown:
            occupied = data >= 50
        else:
            occupied = data != 0
        self._width = width
        self._height = height
        self._resolution = float(msg.info.resolution)
        self._origin_x = float(msg.info.origin.position.x)
        self._origin_y = float(msg.info.origin.position.y)
        self._blocked = self._inflate(occupied)
        if not self._have_map:
            self._have_map = True
            self.get_logger().info(
                f'收到地图 {width}x{height}，分辨率 {self._resolution} m/格')

    def _on_pose(self, msg):
        """记录当前位姿，作为每次规划的起点。"""
        self._start = (float(msg.pose.position.x), float(msg.pose.position.y))

    def _on_goal(self, msg):
        """收到新目标就置待规划标志，真正的搜索放到定时器里，避免阻塞回调。"""
        self._goal = (float(msg.pose.position.x), float(msg.pose.position.y))
        self._pending = True
        self.get_logger().info(f'收到目标点 ({self._goal[0]:.2f}, {self._goal[1]:.2f})')

    # ------------------------------------------------------------------ 膨胀
    def _inflate(self, occupied):
        """按半径膨胀障碍，用逐偏移平移实现，避免引入 scipy 依赖。

        膨胀的物理含义：无人机有安全半径，贴着墙飞会撞，所以把障碍向外扩一圈。
        """
        radius_cells = self._inflation / self._resolution
        if radius_cells <= 0.0:
            return occupied
        reach = int(math.ceil(radius_cells))
        inflated = occupied.copy()
        limit = radius_cells * radius_cells
        for dy in range(-reach, reach + 1):
            for dx in range(-reach, reach + 1):
                if dx == 0 and dy == 0:
                    continue
                if dx * dx + dy * dy > limit:
                    continue
                shifted = np.zeros_like(occupied)
                src_y0 = max(0, -dy)
                src_y1 = min(self._height, self._height - dy)
                src_x0 = max(0, -dx)
                src_x1 = min(self._width, self._width - dx)
                if src_y0 >= src_y1 or src_x0 >= src_x1:
                    continue
                shifted[src_y0 + dy:src_y1 + dy, src_x0 + dx:src_x1 + dx] = \
                    occupied[src_y0:src_y1, src_x0:src_x1]
                inflated |= shifted
        return inflated

    # ------------------------------------------------------------------ 规划
    def _try_plan(self):
        """有待处理目标时执行一次规划。"""
        if not self._pending or not self._have_map or self._start is None:
            return
        self._pending = False
        start_cell = world_to_grid(
            self._start[0], self._start[1], self._origin_x, self._origin_y, self._resolution)
        goal_cell = world_to_grid(
            self._goal[0], self._goal[1], self._origin_x, self._origin_y, self._resolution)

        goal_cell = self._nearest_free(goal_cell)
        if goal_cell is None:
            self.get_logger().error('目标点周围没有可通行的栅格，放弃本次规划')
            return
        # 无人机自己的位置常常正好压在膨胀区里（机身贴着地面时的深度点云
        # 会把脚下标成障碍），起点也要就近挪到可通行栅格，否则每次都不可达。
        free_start = self._nearest_free(start_cell)
        if free_start is None:
            self.get_logger().error('起点周围没有可通行的栅格，放弃本次规划')
            return
        if free_start != start_cell:
            self.get_logger().warn(
                f'起点位于障碍/膨胀区内，改用最近的可通行栅格 {free_start}')
            start_cell = free_start

        begin = time.monotonic()
        cells = self._astar(start_cell, goal_cell)
        elapsed_ms = (time.monotonic() - begin) * 1000.0
        if not cells:
            self.get_logger().error(
                f'规划失败：{start_cell} -> {goal_cell} 不可达，耗时 {elapsed_ms:.1f} ms')
            return

        length = 0.0
        for first, second in zip(cells, cells[1:]):
            length += math.hypot(second[0] - first[0], second[1] - first[1]) * self._resolution
        self.get_logger().info(
            f'规划成功：{len(cells)} 个栅格，长度 {length:.2f} m，耗时 {elapsed_ms:.1f} ms')
        self._publish_path(cells)

    def _nearest_free(self, cell):
        """目标落在障碍上时，在 10 格内找最近的可通行格子。"""
        if not self._is_blocked(cell):
            return cell
        for ring in range(1, 11):
            for dy in range(-ring, ring + 1):
                for dx in range(-ring, ring + 1):
                    if max(abs(dx), abs(dy)) != ring:
                        continue
                    candidate = (cell[0] + dx, cell[1] + dy)
                    if not self._is_blocked(candidate):
                        return candidate
        return None

    def _is_blocked(self, cell):
        """判断栅格是否越界或被占据。"""
        x, y = cell
        if x < 0 or y < 0 or x >= self._width or y >= self._height:
            return True
        return bool(self._blocked[y, x])

    def _astar(self, start, goal):
        """8 邻域 A*：f(n) = g(n) + h(n)，对角代价 sqrt(2)，并检查对角穿角。

        返回从起点到终点的栅格列表；不可达时返回空列表。
        """
        if self._is_blocked(start) and start != goal:
            return []
        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]
        if self._allow_diagonal:
            diagonal = math.sqrt(2.0)
            moves += [(1, 1, diagonal), (1, -1, diagonal),
                      (-1, 1, diagonal), (-1, -1, diagonal)]

        def heuristic(cell):
            return math.hypot(cell[0] - goal[0], cell[1] - goal[1])

        open_heap = [(heuristic(start), 0.0, start)]
        came_from = {}
        best_cost = {start: 0.0}
        closed = set()
        expansions = 0

        while open_heap:
            _, cost, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == goal:
                return self._reconstruct(came_from, current)
            closed.add(current)
            expansions += 1
            if expansions > self._max_expansions:
                self.get_logger().error('A* 扩展次数超限，放弃本次规划')
                return []

            for dx, dy, step in moves:
                neighbour = (current[0] + dx, current[1] + dy)
                if self._is_blocked(neighbour):
                    continue
                if dx != 0 and dy != 0:
                    # 对角穿角检查：两条直角边都必须是空的，否则会“擦着墙角”穿过去
                    if self._is_blocked((current[0] + dx, current[1])):
                        continue
                    if self._is_blocked((current[0], current[1] + dy)):
                        continue
                new_cost = cost + step
                if new_cost < best_cost.get(neighbour, float('inf')):
                    best_cost[neighbour] = new_cost
                    came_from[neighbour] = current
                    heapq.heappush(
                        open_heap, (new_cost + heuristic(neighbour), new_cost, neighbour))
        return []

    @staticmethod
    def _reconstruct(came_from, current):
        """回溯 came_from 得到正向路径。"""
        cells = [current]
        while current in came_from:
            current = came_from[current]
            cells.append(current)
        cells.reverse()
        return cells

    def _publish_path(self, cells):
        """把栅格路径转成世界坐标的 nav_msgs/Path 并发布。"""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for cell_x, cell_y in cells:
            world_x, world_y = grid_to_world(
                cell_x, cell_y, self._origin_x, self._origin_y, self._resolution)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self._path_pub.publish(msg)
