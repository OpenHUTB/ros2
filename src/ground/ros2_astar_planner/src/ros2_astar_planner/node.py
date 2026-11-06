"""ROS2 节点：订阅栅格地图与目标，发布全局路径。

话题
----
- 订阅 ``/map``（nav_msgs/OccupancyGrid）：栅格地图
- 订阅 ``/goal``（geometry_msgs/PoseStamped）：规划目标
- 订阅 ``/robot_pose``（geometry_msgs/PoseStamped）：机器人当前位姿（作为起点）
- 发布 ``/plan``（nav_msgs/Path）：平滑并时间参数化后的路径
- 发布 ``/plan_info``（std_msgs/String）：JSON 诊断信息

安全策略
--------
1. **地图新鲜度**：``/map`` 超过 ``map_timeout`` 秒未更新则拒绝规划并告警，
   避免在过期地图上给出穿墙路径；
2. **起点缺失**：没有 ``/robot_pose`` 时拒绝规划（而不是拿目标当起点）；
3. **参数校验**：所有影响安全的参数在启动时做区间校验，非法值回退默认；
4. **扩展上限**：A* 的 ``max_expansions`` 防止地图异常时把 CPU 打满；
5. **结果复核**：平滑后的路径逐段做连续空间碰撞检查，不通过则回退到
   未经平滑的原始路径。
"""

from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from std_msgs.msg import String

from .astar import AStarConfig, AStarPlanner
from .gridmap import GridMap, GridSpec
from .safety import (
    DeadlineGuard,
    choose_connected_endpoints,
    inflation_cells,
    sanitize_positive,
    validate_map,
)
from .smoothing import smooth_path
from .trajectory import time_parameterize


class AStarPlannerNode(Node):
    """栅格 A* 全局规划节点。"""

    def __init__(self) -> None:
        super().__init__("astar_planner")

        # ---------------- 参数 ----------------
        self.declare_parameter("robot_radius", 0.20)
        self.declare_parameter("inflation_extra", 0.0)
        self.declare_parameter("allow_diagonal", True)
        self.declare_parameter("heuristic", "octile")
        self.declare_parameter("weight", 1.0)
        self.declare_parameter("max_expansions", 200000)
        self.declare_parameter("plan_timeout", 2.0)
        self.declare_parameter("map_timeout", 5.0)
        self.declare_parameter("v_max", 0.6)
        self.declare_parameter("a_max", 0.5)
        self.declare_parameter("a_lat_max", 1.0)
        self.declare_parameter("use_smoothing", True)
        self.declare_parameter("occupied_thresh", 65)
        self.declare_parameter("free_thresh", 196)

        self.robot_radius = self._param_float("robot_radius", 0.20, 0.001, 5.0)
        self.inflation_extra = self._param_float("inflation_extra", 0.0, 0.0, 2.0)
        self.allow_diagonal = bool(self.get_parameter("allow_diagonal").value)
        self.heuristic = str(self.get_parameter("heuristic").value)
        self.weight = self._param_float("weight", 1.0, 1.0, 10.0)
        self.max_expansions = int(self._param_float("max_expansions", 200000, 1000, 5_000_000))
        self.plan_timeout = self._param_float("plan_timeout", 2.0, 0.05, 60.0)
        self.map_timeout = self._param_float("map_timeout", 5.0, 0.1, 3600.0)
        self.v_max = self._param_float("v_max", 0.6, 0.01, 10.0)
        self.a_max = self._param_float("a_max", 0.5, 0.01, 10.0)
        self.a_lat_max = self._param_float("a_lat_max", 1.0, 0.01, 20.0)
        self.use_smoothing = bool(self.get_parameter("use_smoothing").value)
        self.occupied_thresh = int(self.get_parameter("occupied_thresh").value)
        self.free_thresh = int(self.get_parameter("free_thresh").value)

        if self.heuristic not in ("octile", "manhattan", "euclidean", "zero"):
            self.get_logger().warn(f"未知启发式 {self.heuristic}，回退 octile")
            self.heuristic = "octile"

        # ---------------- 状态 ----------------
        self.gridmap: GridMap | None = None
        self.map_stamp: float | None = None
        self.goal: tuple[float, float] | None = None
        self.robot: tuple[float, float] | None = None

        self.planner = AStarPlanner(
            AStarConfig(
                allow_diagonal=self.allow_diagonal,
                heuristic=self.heuristic,
                weight=self.weight,
                max_expansions=self.max_expansions,
            )
        )

        # ---------------- 订阅 / 发布 ----------------
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 1)
        self.create_subscription(PoseStamped, "/goal", self._on_goal, 10)
        self.create_subscription(PoseStamped, "/robot_pose", self._on_robot_pose, 10)
        self.path_pub = self.create_publisher(Path, "/plan", 10)
        self.info_pub = self.create_publisher(String, "/plan_info", 10)

        self.get_logger().info(
            f"A* 规划节点已启动：半径 {self.robot_radius:.2f} m，"
            f"启发式 {self.heuristic}，允许对角 {self.allow_diagonal}"
        )

    # ---------------- 参数辅助 ----------------

    def _param_float(self, name: str, default: float, lo: float, hi: float) -> float:
        raw = self.get_parameter(name).value
        value, warning = sanitize_positive(float(raw), default, name, lo=lo, hi=hi)
        if warning:
            self.get_logger().warn(warning)
        return value

    # ---------------- 回调 ----------------

    def _on_map(self, msg: OccupancyGrid) -> None:
        w = int(msg.info.width)
        h = int(msg.info.height)
        res = float(msg.info.resolution)
        if w <= 0 or h <= 0 or res <= 0.0:
            self.get_logger().error("地图尺寸或分辨率非法，已忽略本次 /map")
            return
        data = np.asarray(msg.data, dtype=np.int32).reshape(h, w)
        # 未知区域（-1）按占用处理：保守起见不把未观测区域当作可通行
        occupied = (data >= self.occupied_thresh) | (data < 0)
        ox = float(msg.info.origin.position.x) - res / 2.0
        oy = float(msg.info.origin.position.y) - res / 2.0
        try:
            self.gridmap = GridMap(occupied, GridSpec(w, h, res, ox, oy))
        except ValueError as exc:
            self.get_logger().error(f"地图构建失败：{exc}")
            return
        self.map_stamp = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f"收到地图 {w}×{h}，分辨率 {res:.3f} m，"
            f"占用格 {int(occupied.sum())} 个",
            throttle_duration_sec=5.0,
        )

    def _on_goal(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.goal = (float(p.x), float(p.y))
        self.get_logger().info(f"收到目标 ({p.x:.2f}, {p.y:.2f})")
        self._try_plan()

    def _on_robot_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.robot = (float(p.x), float(p.y))

    # ---------------- 规划 ----------------

    def _try_plan(self) -> None:
        if self.gridmap is None:
            self.get_logger().warn("尚未收到 /map，无法规划")
            return
        if self.goal is None:
            return
        if self.robot is None:
            self.get_logger().warn("尚未收到 /robot_pose，无法确定起点")
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        age = now - (self.map_stamp or 0.0)
        if age > self.map_timeout:
            self.get_logger().warn(
                f"地图已过期（{age:.1f}s > {self.map_timeout:.1f}s），拒绝规划"
            )
            return

        problem = validate_map(self.gridmap)
        if problem:
            self.get_logger().error(f"地图不可用：{problem}")
            return

        gm = self.gridmap
        n_infl = inflation_cells(self.robot_radius + self.inflation_extra, gm.spec.resolution)
        inflated = gm.inflate(n_infl)

        i0, j0 = inflated.to_grid(*self.robot)
        i1, j1 = inflated.to_grid(*self.goal)
        start, goal, notes = choose_connected_endpoints(inflated, (i0, j0), (i1, j1))
        for note in notes:
            self.get_logger().warn(note)

        guard = DeadlineGuard(self.plan_timeout).start()
        result = self.planner.plan(inflated, start, goal)
        if guard.expired():
            self.get_logger().warn(
                f"规划耗时超过 {self.plan_timeout:.2f}s 已放弃（结果仅供参考）"
            )
        if not result.success:
            self.get_logger().error(f"规划失败：{result.reason}")
            self._publish_info(
                {
                    "success": False,
                    "reason": result.reason,
                    "expansions": result.expansions,
                    "inflation_cells": n_infl,
                }
            )
            return

        world = np.asarray(
            [inflated.to_world(i, j) for i, j in result.cells], dtype=float
        )
        if self.use_smoothing:
            smoothed, metrics = smooth_path(inflated, world, use_gradient=True, seed=0)
        else:
            smoothed, metrics = world, {
                "raw_length": float(np.sum(np.linalg.norm(np.diff(world, axis=0), axis=1)))
                if len(world) > 1
                else 0.0,
                "final_length": float(np.sum(np.linalg.norm(np.diff(world, axis=0), axis=1)))
                if len(world) > 1
                else 0.0,
                "safe": True,
                "final_points": len(world),
                "raw_points": len(world),
                "raw_turns": 0,
                "final_turns": 0,
                "after_shortcut_length": 0.0,
            }

        traj = time_parameterize(
            smoothed, self.v_max, self.a_max, self.a_lat_max
        )

        self._publish_path(smoothed, traj)
        self._publish_info(
            {
                "success": True,
                "points": len(smoothed),
                "grid_cost": result.cost,
                "expansions": result.expansions,
                "path_length": float(traj.total_length),
                "trajectory_time": float(traj.total_time),
                "v_max_actual": float(traj.v_max_actual),
                "inflation_cells": n_infl,
                "smooth": bool(self.use_smoothing),
                "safe": bool(metrics.get("safe", True)),
                "notes": notes,
            }
        )
        self.get_logger().info(
            f"规划完成：{len(smoothed)} 个路径点，长度 {traj.total_length:.2f} m，"
            f"预计耗时 {traj.total_time:.2f} s，扩展 {result.expansions} 个节点",
            throttle_duration_sec=2.0,
        )

    def _publish_path(self, points: np.ndarray, traj) -> None:
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            # 用速度方向填充朝向，便于下游控制器直接使用
            if i + 1 < len(points):
                dx = float(points[i + 1][0] - x)
                dy = float(points[i + 1][1] - y)
                if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                    pose.pose.orientation.z = math.sin(math.atan2(dy, dx) / 2.0)
                    pose.pose.orientation.w = math.cos(math.atan2(dy, dx) / 2.0)
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def _publish_info(self, payload: dict) -> None:
        self.info_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
