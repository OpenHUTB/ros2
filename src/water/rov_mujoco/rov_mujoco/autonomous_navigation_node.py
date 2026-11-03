#!/usr/bin/env python3
"""
水下机器人多航点自主导航与 SLAM 同步闭环系统 (Autonomous Navigation Node)
====================================================================
功能：
  1. 维护水下多任务航点队列 (Waypoints Mission Executive)
  2. 边建图边导航 (Simultaneous SLAM & Autonomous Navigation / Explore-and-Navigate)
  3. 结合 Neural A* 全局路径规划与深度强化学习局部声呐避障策略
  4. 广播 /rov/global_path, /rov/local_path, /rov/current_goal, /rov/avoidance_count
  5. 统计巡航耗时、避障次数、航点到达率与终点到位精度

本文件同时提供与 ROS 解耦的 NavigationCore：
  - ROS 节点、main.py 的 GUI 演示、test_task3.py 的闭环测试共用同一套导航逻辑，
    避免“测试里一套、演示里另一套”导致结果不可复现。
"""

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32


class NavigationCore:
    """建图—全局规划—局部避障闭环导航核心 (不依赖 ROS，便于单元测试与 GUI 复用)"""

    def __init__(
        self,
        waypoints,
        planner_type: str = 'neural',
        max_speed: float = 0.55,
        tolerance_reach: float = 0.25,
        final_tolerance: float = 0.10,
        depth_tolerance: float = 0.35,
        replan_period: float = 1.0,
        planner_resolution: float = 0.1,
        clearance_m: float = 0.35,
    ):
        try:
            from rov_mujoco.neural_path_planner import (
                NeuralAStarPlanner, BaselineAStarPlanner,
                NeuralLocalAvoidancePolicy, BaselineDWAPlanner,
            )
        except ImportError:
            from neural_path_planner import (
                NeuralAStarPlanner, BaselineAStarPlanner,
                NeuralLocalAvoidancePolicy, BaselineDWAPlanner,
            )

        self.waypoints = [tuple(float(v) for v in wp) for wp in waypoints]
        self.planner_type = planner_type
        self.max_speed = float(max_speed)
        self.tolerance_reach = float(tolerance_reach)
        self.final_tolerance = float(final_tolerance)
        self.depth_tolerance = float(depth_tolerance)
        self.replan_period = float(replan_period)
        self.planner_resolution = float(planner_resolution)

        if planner_type == 'neural':
            self.global_planner = NeuralAStarPlanner(
                resolution=planner_resolution, clearance_m=clearance_m)
            self.local_planner = NeuralLocalAvoidancePolicy(
                safe_margin=0.65, max_speed=max_speed)
        else:
            self.global_planner = BaselineAStarPlanner(
                resolution=planner_resolution, clearance_m=clearance_m)
            self.local_planner = BaselineDWAPlanner(
                max_speed=max_speed, safe_margin=0.65)

        self.wp_idx = 0
        self.completed = False
        self.global_path = []          # [(x, y), ...]
        self.local_path = []
        self.avoidance_count = 0
        self.total_distance = 0.0
        self.last_plan_time = -1e9
        self.last_plan_ms = 0.0
        self.last_expanded_nodes = 0

        self._grid = None              # 规划用降采样占据栅格 (0/100)
        self._origin = (0.0, 0.0)
        self._grid_res = planner_resolution
        self._map_version = 0
        self._planned_version = -1
        self._planned_wp = -1
        self._last_pos = None
        self._last_avoid_time = -1e9
        self._in_avoidance = False

    # ── 地图 ────────────────────────────────────────────────
    def set_map(self, grid, origin, resolution):
        """接收 OccupancyGrid 数据，降采样到规划分辨率（未知区域视为可通行）"""
        grid = np.asarray(grid, dtype=np.int16)
        h, w = grid.shape
        factor = max(1, int(round(self.planner_resolution / max(resolution, 1e-6))))
        if factor > 1:
            h2, w2 = h // factor, w // factor
            if h2 == 0 or w2 == 0:
                return
            blocks = grid[:h2 * factor, :w2 * factor].reshape(h2, factor, w2, factor)
            coarse = np.where((blocks >= 50).any(axis=(1, 3)), 100, 0).astype(np.int16)
        else:
            coarse = np.where(grid >= 50, 100, 0).astype(np.int16)
            factor = 1

        self._grid = coarse
        self._origin = (float(origin[0]), float(origin[1]))
        self._grid_res = float(resolution) * factor
        self._map_version += 1

    def _world_to_map(self, x, y):
        mx = int(math.floor((x - self._origin[0]) / self._grid_res))
        my = int(math.floor((y - self._origin[1]) / self._grid_res))
        if 0 <= mx < self._grid.shape[1] and 0 <= my < self._grid.shape[0]:
            return mx, my
        return None

    def _cell_blocked(self, x, y):
        if self._grid is None:
            return False
        mx, my = self._world_to_map(x, y) or (None, None)
        if mx is None:
            return False
        return bool(self._grid[my, mx] >= 50)

    # ── 全局规划 ────────────────────────────────────────────
    def _need_replan(self, now, target_wp):
        if not self.global_path:
            return True
        if self._planned_wp != self.wp_idx:
            return True
        if self._planned_version != self._map_version and \
                now - self.last_plan_time >= self.replan_period:
            return True
        return False

    def _replan(self, start_xy, target_wp, now):
        t0 = time.perf_counter()
        if self._grid is None:
            self.global_path = [tuple(start_xy), (float(target_wp[0]), float(target_wp[1]))]
            self.last_expanded_nodes = 0
        else:
            path = self.global_planner.plan(
                self._grid, self._origin, tuple(start_xy),
                (float(target_wp[0]), float(target_wp[1])),
            )
            if len(path) < 2:
                path = [tuple(start_xy), (float(target_wp[0]), float(target_wp[1]))]
            self.global_path = [(float(p[0]), float(p[1])) for p in path]
            self.last_expanded_nodes = int(getattr(self.global_planner, 'expanded_nodes', 0))
        self.last_plan_ms = (time.perf_counter() - t0) * 1000.0
        self.last_plan_time = now
        self._planned_wp = self.wp_idx
        self._planned_version = self._map_version

    def _lookahead_target(self, pos_xy):
        """沿全局路径取前方 0.6~0.9 m 的跟踪点，避免直接切直线撞向障碍"""
        if not self.global_path:
            return None
        target = self.global_path[-1]
        for p in self.global_path:
            if math.hypot(p[0] - pos_xy[0], p[1] - pos_xy[1]) >= 0.6:
                target = p
                break
        return target

    # ── 局部安全 ────────────────────────────────────────────
    def _costmap_ahead_blocked(self, pos, yaw, vx, vy, wz, horizon=0.6):
        if self._grid is None:
            return False
        for t in (0.2, 0.4, 0.6)[:max(1, int(horizon / 0.2))]:
            psi = yaw + wz * t
            x = pos[0] + (vx * math.cos(psi) - vy * math.sin(psi)) * t
            y = pos[1] + (vx * math.sin(psi) + vy * math.cos(psi)) * t
            if self._cell_blocked(x, y):
                return True
        return False

    @staticmethod
    def _predict_local_path(pos, yaw, vx, vy, wz, steps=12, dt=0.1):
        x, y, psi = float(pos[0]), float(pos[1]), float(yaw)
        path = [(x, y)]
        for _ in range(steps):
            x += (vx * math.cos(psi) - vy * math.sin(psi)) * dt
            y += (vx * math.sin(psi) + vy * math.cos(psi)) * dt
            psi += wz * dt
            path.append((x, y))
        return path

    # ── 主更新 ──────────────────────────────────────────────
    def update(self, pos, yaw, vel, sonar_ranges, now=None):
        """
        Args:
            pos: [x, y, z] 世界坐标
            yaw: 偏航角 (rad)
            vel: 世界系线速度 [vx, vy, vz]
            sonar_ranges: 72 束声呐距离
            now: 单调时间 (默认 time.monotonic)
        Returns dict: vx, vy, wz, target_wp, wp_idx, completed, in_avoidance,
                      replanned, global_path, local_path, dist_to_wp
        """
        if now is None:
            now = time.monotonic()

        pos = np.asarray(pos, dtype=float)
        if self._last_pos is not None:
            self.total_distance += float(np.linalg.norm(pos[:2] - self._last_pos[:2]))
        self._last_pos = pos.copy()

        if self.completed or self.wp_idx >= len(self.waypoints):
            self.completed = True
            return self._result(0.0, 0.0, 0.0, False,
                                self.waypoints[-1], [tuple(pos[:2])])

        target_wp = self.waypoints[self.wp_idx]
        dist_2d = math.hypot(target_wp[0] - pos[0], target_wp[1] - pos[1])
        depth_err = abs(target_wp[2] - pos[2])

        # 航点到达判定：中间航点用宽阈值，最后一个航点收紧到终点对接精度
        is_final_wp = (self.wp_idx == len(self.waypoints) - 1)
        arrive_tol = self.final_tolerance if is_final_wp else self.tolerance_reach
        if dist_2d < arrive_tol and depth_err < self.depth_tolerance:
            self.wp_idx += 1
            self.global_path = []
            if self.wp_idx >= len(self.waypoints):
                self.completed = True
                return self._result(0.0, 0.0, 0.0, False,
                                    target_wp, [tuple(pos[:2])])
            target_wp = self.waypoints[self.wp_idx]
            dist_2d = math.hypot(target_wp[0] - pos[0], target_wp[1] - pos[1])

        # 全局规划（首次 / 航点切换 / 地图更新后限频重规划）
        replanned = False
        if self._need_replan(now, target_wp):
            self._replan(tuple(pos[:2]), target_wp, now)
            replanned = True

        track_target = self._lookahead_target(tuple(pos[:2])) or \
            (float(target_wp[0]), float(target_wp[1]))

        # 局部神经网络避障策略
        vx_body = (math.cos(yaw) * vel[0] + math.sin(yaw) * vel[1]) if vel is not None else 0.0
        plan = self.local_planner.compute_cmd(
            sonar_ranges=sonar_ranges,
            current_pos=(float(pos[0]), float(pos[1])),
            current_yaw=float(yaw),
            target_wp=track_target,
            current_vx=float(vx_body),
        )

        vx, vy, wz = plan["vx"], plan["vy"], plan["wz"]

        # 终点精确对接：0.8 m 内按距离比例减速，避免以巡航速度冲过目标点
        if is_final_wp and dist_2d < 0.8:
            vx = min(vx, max(0.06, 1.0 * dist_2d))
            vy *= max(0.15, min(1.0, dist_2d / 0.8))

        # 代价地图前向安全检查（把 /map 真正用于局部控制）
        blocked_ahead = self._costmap_ahead_blocked(pos, yaw, vx, vy, wz)
        in_avoidance = bool(plan.get("in_avoidance", False)) or blocked_ahead
        if blocked_ahead:
            vx = 0.0
            wz = float(np.clip(wz + (0.45 if not self._cell_blocked(
                pos[0] + 0.4 * math.cos(yaw + 0.6),
                pos[1] + 0.4 * math.sin(yaw + 0.6)) else -0.45), -0.6, 0.6))

        # 避障次数：按“避障事件”边沿计数，而非每个控制 tick 累加
        if in_avoidance:
            if now - self._last_avoid_time > 1.0:
                self.avoidance_count += 1
            self._last_avoid_time = now
        self._in_avoidance = in_avoidance

        self.local_path = self._predict_local_path(pos, yaw, vx, vy, wz)
        return self._result(vx, vy, wz, in_avoidance, target_wp, self.local_path,
                            replanned=replanned, dist_to_wp=dist_2d)

    def _result(self, vx, vy, wz, in_avoidance, target_wp, local_path,
                replanned=False, dist_to_wp=0.0):
        return {
            "vx": float(vx),
            "vy": float(vy),
            "wz": float(wz),
            "target_wp": target_wp,
            "wp_idx": self.wp_idx,
            "completed": self.completed,
            "in_avoidance": bool(in_avoidance),
            "replanned": bool(replanned),
            "global_path": list(self.global_path),
            "local_path": list(local_path),
            "dist_to_wp": float(dist_to_wp),
            "planning_time_ms": float(self.last_plan_ms),
            "expanded_nodes": int(self.last_expanded_nodes),
        }

    def summary(self, final_pos=None):
        total = len(self.waypoints)
        if final_pos is not None and total:
            final_err = float(math.hypot(
                self.waypoints[-1][0] - final_pos[0],
                self.waypoints[-1][1] - final_pos[1]))
        else:
            final_err = 0.0
        return {
            "completed": self.completed,
            "waypoints_reached": self.wp_idx,
            "total_waypoints": total,
            "total_distance_m": float(self.total_distance),
            "avoidance_triggers": int(self.avoidance_count),
            "final_position_error_m": final_err,
            "success_rate_pct": (self.wp_idx / total) * 100.0 if total else 0.0,
        }


class AutonomousNavigationNode(Node):
    """水下自主多航点导航控制节点"""

    def __init__(self):
        super().__init__('autonomous_navigation_node')

        self.declare_parameter('tolerance_reach', 0.25)
        self.declare_parameter('max_linear_speed', 0.55)
        self.declare_parameter('planner_type', 'neural')  # neural 或 baseline
        self.declare_parameter('replan_period', 1.0)

        tolerance = self.get_parameter('tolerance_reach').value
        max_speed = self.get_parameter('max_linear_speed').value
        planner_type = self.get_parameter('planner_type').value
        replan_period = self.get_parameter('replan_period').value

        # 水下管网作业区 3D 巡检任务航点序列 [X, Y, Z]
        self.waypoints = [
            (-1.2,  0.0, -1.5),
            ( 0.0,  1.2, -1.5),
            ( 0.3, -0.2, -1.5),
            (-0.5, -1.2, -1.5),
        ]
        self.core = NavigationCore(
            self.waypoints,
            planner_type=planner_type,
            max_speed=max_speed,
            tolerance_reach=tolerance,
            replan_period=replan_period,
        )

        # 通信订阅
        self.sub_odom = self.create_subscription(Odometry, '/rov/odom', self._odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/rov/sonar/scan', self._scan_callback, 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self._map_callback, 1)

        # 控制与路径发布
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_global_path = self.create_publisher(Path, '/rov/global_path', 1)
        self.pub_local_path = self.create_publisher(Path, '/rov/local_path', 1)
        self.pub_goal = self.create_publisher(PoseStamped, '/rov/current_goal', 1)
        self.pub_avoidance = self.create_publisher(Int32, '/rov/avoidance_count', 1)

        # 缓存状态
        self.current_pos = np.array([0.0, 0.0, -1.0])
        self.current_yaw = 0.0
        self.current_vel = np.array([0.0, 0.0, 0.0])
        self.sonar_ranges = [15.0] * 72
        self.has_odom = False
        self._last_path_pub = 0.0
        self._last_avoid_pub = -1
        self._last_avoid_pub_time = 0.0

        # 50 Hz 导航决策循环
        self.nav_timer = self.create_timer(0.02, self._navigation_step)

        self.get_logger().info(
            f'水下多航点自主导航节点已启动 | 规划器模式: {planner_type.upper()} | '
            f'目标航点数: {len(self.waypoints)}'
        )

    def _odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.current_pos = np.array([p.x, p.y, p.z])

        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))

        v = msg.twist.twist.linear
        self.current_vel = np.array([v.x, v.y, v.z])
        self.has_odom = True

    def _scan_callback(self, msg: LaserScan):
        self.sonar_ranges = list(msg.ranges)

    def _map_callback(self, msg: OccupancyGrid):
        h, w = msg.info.height, msg.info.width
        grid = np.array(msg.data, dtype=np.int16).reshape((h, w))
        origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.core.set_map(grid, origin, msg.info.resolution)

    def _make_path_msg(self, points, z=0.0):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for x, y in points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(z)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    def _navigation_step(self):
        """核心导航控制状态机"""
        if not self.has_odom:
            return

        res = self.core.update(
            self.current_pos, self.current_yaw, self.current_vel,
            self.sonar_ranges, now=time.monotonic(),
        )

        cmd = Twist()
        if not res["completed"]:
            cmd.linear.x = res["vx"]
            cmd.linear.y = res["vy"]
            target_wp = res["target_wp"]
            dz = target_wp[2] - self.current_pos[2]
            cmd.linear.z = float(np.clip(1.8 * dz, -0.4, 0.4))
            cmd.angular.z = res["wz"]
        self.pub_cmd_vel.publish(cmd)

        # 目标位姿
        target_wp = res["target_wp"]
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = float(target_wp[0])
        goal_msg.pose.position.y = float(target_wp[1])
        goal_msg.pose.position.z = float(target_wp[2])
        goal_msg.pose.orientation.w = 1.0
        self.pub_goal.publish(goal_msg)

        # 路径话题：全局路径 1 Hz 重发（规划时立即发布），局部预测轨迹 5 Hz
        now = time.monotonic()
        if res["global_path"] and (res["replanned"] or now - self._last_path_pub >= 1.0):
            self.pub_global_path.publish(self._make_path_msg(res["global_path"]))
            self._last_path_pub = now
        self.pub_local_path.publish(self._make_path_msg(res["local_path"]))

        # 避障计数（变化时立即发布，另以 0.5 Hz 重发供监控端订阅）
        if self.core.avoidance_count != self._last_avoid_pub or \
                now - self._last_avoid_pub_time >= 2.0:
            self.pub_avoidance.publish(Int32(data=int(self.core.avoidance_count)))
            self._last_avoid_pub = self.core.avoidance_count
            self._last_avoid_pub_time = now

    def get_summary(self) -> dict:
        """获取自主导航性能评价指标"""
        return self.core.summary(final_pos=self.current_pos)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavigationNode()
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
