# Copyright 2026 OpenHUTB
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See http://www.apache.org/licenses/LICENSE-2.0 for the full text.
"""A* 全局路径规划节点。

订阅 ``/map``（占据栅格）与 ``/goal_pose``（目标点），在栅格上做 8 邻域 A*，
对占据栅格按 ``inflation_radius`` 膨胀出安全边界，结果以 ``nav_msgs/Path``
发布到 ``/plan`` 供控制节点跟踪。
"""

import math
import time

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from uav_depth_mapping_nav import astar as astar_lib
from uav_depth_mapping_nav.grid_utils import grid_to_world, world_to_grid


class Planner(Node):
    """占据栅格上的 A* 规划器。"""

    def __init__(self):
        super().__init__('planner', namespace='uav')

        self.declare_parameter('inflation_radius', 0.6)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('allow_unknown', True)
        # 未知栅格按“可通行但代价更高”处理：既不把没扫到的地方当墙，
        # 也避免 A* 为了抄近路穿过大片未知区域。
        self.declare_parameter('unknown_penalty', 4.0)
        # 收到目标后每隔这么久用最新地图重规划一次（地图是边飞边建的）
        self.declare_parameter('replan_period', 3.0)
        self.declare_parameter('plan_period', 0.5)
        self.declare_parameter('max_expansions', 200000)

        self._inflation = float(self.get_parameter('inflation_radius').value)
        self._allow_diagonal = bool(self.get_parameter('allow_diagonal').value)
        self._allow_unknown = bool(self.get_parameter('allow_unknown').value)
        self._unknown_penalty = float(
            self.get_parameter('unknown_penalty').value)
        self._replan_period = float(self.get_parameter('replan_period').value)
        self._last_plan_time = 0.0
        self._goal_active = False
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

        self.create_timer(1.0 / self._plan_period, self._on_timer)
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
        # -1 表示未知；把它单独记下来，A* 里加代价而不是直接当墙
        self._unknown = data < 0
        if not self._have_map:
            self._have_map = True
            self.get_logger().info(
                f'收到地图 {width}x{height}，分辨率 {self._resolution} m/格')

    def _on_pose(self, msg):
        """记录当前位姿，作为每次规划的起点。"""
        self._start = (float(msg.pose.position.x), float(msg.pose.position.y))

    def _on_goal(self, msg):
        """收到新目标就置待规划标志，真正的搜索放到定时器里，避免阻塞回调。"""
        self._goal_active = True
        self._goal = (float(msg.pose.position.x), float(msg.pose.position.y))
        self._pending = True
        self.get_logger().info(f'收到目标点 ({self._goal[0]:.2f}, {self._goal[1]:.2f})')

    # ------------------------------------------------------------------ 膨胀
    def _inflate(self, occupied):
        """按半径膨胀障碍（实现在 astar.py，可在无 ROS 环境下测试）。"""
        return astar_lib.inflate(occupied, self._resolution, self._inflation)

    def _on_timer(self):
        """定时器：先看要不要按新地图重规划，再真正执行一次规划。"""
        self._maybe_replan()
        self._try_plan()

    def _maybe_replan(self):
        """地图是边飞边建的：到达目标前按 replan_period 用最新地图重规划。"""
        if not self._goal_active or not self._have_map or self._start is None:
            return
        if time.monotonic() - self._last_plan_time < self._replan_period:
            return
        self._pending = True

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
        self._last_plan_time = begin
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
        return astar_lib.nearest_free(self._blocked, cell)

    def _is_blocked(self, cell):
        """判断栅格是否越界或被占据。"""
        x, y = cell
        if x < 0 or y < 0 or x >= self._width or y >= self._height:
            return True
        return bool(self._blocked[y, x])

    def _astar(self, start, goal):
        """8 邻域 A*（实现在 astar.py）。

        f(n) = g(n) + h(n)，对角代价 sqrt(2)，对角移动前检查两条直角边；
        未知栅格可通行但步长代价乘 ``unknown_penalty``。
        """
        result = astar_lib.astar(
            self._blocked, start, goal,
            unknown=self._unknown,
            allow_diagonal=self._allow_diagonal,
            allow_unknown=self._allow_unknown,
            unknown_penalty=self._unknown_penalty,
            max_expansions=self._max_expansions)
        if result['reason'] == 'max_expansions':
            self.get_logger().error('A* 扩展次数超限，放弃本次规划')
        elif result['reason'] == 'no_path':
            self.get_logger().warn(
                f'A* 未找到路径 {start} -> {goal}（扩展 {result["expansions"]} 个节点）',
                throttle_duration_sec=2.0)
        elif result['success']:
            self.get_logger().info(
                f'A* 扩展 {result["expansions"]} 个节点，'
                f'路径 {len(result["path"])} 个栅格，耗时 {result["elapsed_ms"]:.2f} ms')
        # 世界坐标长度 = 栅格代价长度 * 分辨率
        result['length_m'] = result['length_cells'] * self._resolution
        self._last_result = result
        return result['path']

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
