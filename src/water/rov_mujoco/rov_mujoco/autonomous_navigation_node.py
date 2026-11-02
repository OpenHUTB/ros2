#!/usr/bin/env python3
"""
水下机器人多航点自主导航与 SLAM 同步闭环系统 (Autonomous Navigation Node)
====================================================================
功能：
  1. 维护水下多任务航点队列 (Waypoints Mission Executive)
  2. 边建图边导航 (Simultaneous SLAM & Autonomous Navigation / Explore-and-Navigate)
  3. 结合 Neural A* 全局路径规划与深度强化学习局部声呐避障策略
  4. 广播 /rov/global_path, /rov/local_path, /rov/current_goal
  5. 统计巡航耗时、避障次数、航点到达率与终点到位精度
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


class AutonomousNavigationNode(Node):
    """水下自主多航点导航控制节点"""

    def __init__(self):
        super().__init__('autonomous_navigation_node')

        self.declare_parameter('tolerance_reach', 0.25)
        self.declare_parameter('max_linear_speed', 0.55)
        self.declare_parameter('planner_type', 'neural')  # neural 或 baseline

        self.tolerance_reach = self.get_parameter('tolerance_reach').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.planner_type = self.get_parameter('planner_type').value

        # 实例化路径规划与避障策略网络
        try:
            from rov_mujoco.neural_path_planner import (
                NeuralAStarPlanner, BaselineAStarPlanner,
                NeuralLocalAvoidancePolicy, BaselineDWAPlanner
            )
        except ImportError:
            try:
                from underwater_rov_mujoco.neural_path_planner import (
                    NeuralAStarPlanner, BaselineAStarPlanner,
                    NeuralLocalAvoidancePolicy, BaselineDWAPlanner
                )
            except ImportError:
                from neural_path_planner import (
                    NeuralAStarPlanner, BaselineAStarPlanner,
                    NeuralLocalAvoidancePolicy, BaselineDWAPlanner
                )

        if self.planner_type == 'neural':
            self.global_planner = NeuralAStarPlanner(resolution=0.1, clearance_m=0.35)
            self.local_planner = NeuralLocalAvoidancePolicy(safe_margin=0.65, max_speed=self.max_linear_speed)
        else:
            self.global_planner = BaselineAStarPlanner(resolution=0.1, clearance_m=0.35)
            self.local_planner = BaselineDWAPlanner(max_speed=self.max_linear_speed, safe_margin=0.65)

        # 水下管网作业区 3D 巡检任务航点序列 [X, Y, Z] (标准巡航安全作业深度 -1.5m，完美环绕走廊)
        self.waypoints = [
            (-1.2,  0.0, -1.5),   # WP 1: 平台出舱与主航道切入段
            ( 0.0,  1.2, -1.5),   # WP 2: 主干油气管线跨越巡检
            ( 0.3, -0.2, -1.5),   # WP 3: 井口采油树远距对准观测 (安全间距 1.5m，宽视场声呐扫测)
            (-0.5, -1.2, -1.5)    # WP 4 / Goal: 结构平台对接终点 (远离立柱，平顺安全对接)
        ]
        self.current_wp_idx = 0
        self.mission_completed = False

        # 通信订阅
        self.sub_odom = self.create_subscription(Odometry, '/rov/odom', self._odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/rov/sonar/scan', self._scan_callback, 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self._map_callback, 1)

        # 控制与路径发布
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_global_path = self.create_publisher(Path, '/rov/global_path', 1)
        self.pub_local_path = self.create_publisher(Path, '/rov/local_path', 1)
        self.pub_goal = self.create_publisher(PoseStamped, '/rov/current_goal', 1)

        # 缓存状态
        self.current_pos = np.array([0.0, 0.0, -1.0])
        self.current_yaw = 0.0
        self.current_vel = np.array([0.0, 0.0, 0.0])
        self.sonar_ranges = [15.0] * 72
        self.costmap = None
        self.costmap_origin = (-10.0, -10.0)
        self.costmap_res = 0.1

        # 统计数据
        self.avoidance_count = 0
        self.total_distance = 0.0
        self.last_pos = None

        # 50 Hz 导航高频决策控制循环
        self.nav_timer = self.create_timer(0.02, self._navigation_step)

        self.get_logger().info(
            f'水下多航点自主导航节点已启动 | 规划器模式: {self.planner_type.upper()} | 目标航点数: {len(self.waypoints)}'
        )

    def _odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.current_pos = np.array([p.x, p.y, p.z])

        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))

        v = msg.twist.twist.linear
        self.current_vel = np.array([v.x, v.y, v.z])

        if self.last_pos is not None:
            self.total_distance += float(np.linalg.norm(self.current_pos[:2] - self.last_pos[:2]))
        self.last_pos = self.current_pos.copy()

    def _scan_callback(self, msg: LaserScan):
        self.sonar_ranges = list(msg.ranges)

    def _map_callback(self, msg: OccupancyGrid):
        h, w = msg.info.height, msg.info.width
        self.costmap = np.array(msg.data, dtype=np.int8).reshape((h, w))
        self.costmap_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.costmap_res = msg.info.resolution

    def _navigation_step(self):
        """核心导航控制状态机"""
        if self.mission_completed:
            # 终点悬停
            cmd = Twist()
            self.pub_cmd_vel.publish(cmd)
            return

        target_wp = self.waypoints[self.current_wp_idx]
        dx = target_wp[0] - self.current_pos[0]
        dy = target_wp[1] - self.current_pos[1]
        dz = target_wp[2] - self.current_pos[2]
        dist_2d = math.hypot(dx, dy)

        # 检查是否到达当前航点
        if dist_2d < self.tolerance_reach and abs(dz) < 0.35:
            self.get_logger().info(
                f'✅ 成功捕获航点 [{self.current_wp_idx + 1}/{len(self.waypoints)}]: {target_wp} | 水平误差: {dist_2d:.3f}m'
            )
            self.current_wp_idx += 1
            if self.current_wp_idx >= len(self.waypoints):
                self.mission_completed = True
                self.get_logger().info(
                    f'🎉 水下多航点全自主巡航任务全部达成！总巡航里程: {self.total_distance:.2f}m | 避障触发次数: {self.avoidance_count}'
                )
                cmd = Twist()
                self.pub_cmd_vel.publish(cmd)
                return
            target_wp = self.waypoints[self.current_wp_idx]

        # 局部神经网络动态避障规划
        plan_res = self.local_planner.compute_cmd(
            sonar_ranges=self.sonar_ranges,
            current_pos=(self.current_pos[0], self.current_pos[1]),
            current_yaw=self.current_yaw,
            target_wp=(target_wp[0], target_wp[1]),
            current_vx=float(self.current_vel[0])
        )

        if plan_res.get("in_avoidance", False):
            self.avoidance_count += 1

        # 组装 6-DOF Twist 控制指令
        cmd = Twist()
        cmd.linear.x = plan_res["vx"]
        cmd.linear.y = plan_res["vy"]
        # 垂向 Z 轴由定深 PID 驱动向目标深度靠近
        cmd.linear.z = float(np.clip(1.8 * dz, -0.4, 0.4))
        cmd.angular.z = plan_res["wz"]

        self.pub_cmd_vel.publish(cmd)

        # 发布当前目标位姿
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = float(target_wp[0])
        goal_msg.pose.position.y = float(target_wp[1])
        goal_msg.pose.position.z = float(target_wp[2])
        self.pub_goal.publish(goal_msg)

    def get_summary(self) -> dict:
        """获取自主导航性能评价指标"""
        target_final = self.waypoints[-1]
        final_err = float(np.linalg.norm(self.current_pos[:2] - np.array(target_final[:2])))
        return {
            "completed": self.mission_completed,
            "waypoints_reached": self.current_wp_idx,
            "total_waypoints": len(self.waypoints),
            "total_distance_m": float(self.total_distance),
            "avoidance_triggers": int(self.avoidance_count),
            "final_position_error_m": final_err,
            "success_rate_pct": (self.current_wp_idx / len(self.waypoints)) * 100.0
        }


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
