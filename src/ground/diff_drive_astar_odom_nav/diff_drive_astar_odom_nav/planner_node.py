#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A* 静态全局规划节点。

订阅 /map（静态占用栅格）与 /odom（起点来自里程计反馈），收到目标点后
在膨胀后的栅格上搜索，发布 /plan（nav_msgs/Path）与 JSON 诊断信息。
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from diff_drive_astar_odom_nav.astar import find_path, inflate_grid, path_length_world
from diff_drive_astar_odom_nav.map_utils import grid_from_occupancy


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PlannerNode(Node):
    def __init__(self):
        super().__init__("astar_planner")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("info_topic", "/plan_info")
        self.declare_parameter("robot_radius", 0.22)
        self.declare_parameter("inflation_extra", 0.05)
        self.declare_parameter("allow_diagonal", True)
        self.declare_parameter("weight", 1.0)
        self.declare_parameter("max_expansions", 200000)
        self.declare_parameter("auto_goal_x", float("nan"))
        self.declare_parameter("auto_goal_y", float("nan"))

        self.resolution = None
        self.origin = (0.0, 0.0)
        self.grid = None
        self.inflated = None
        self.start_pose = None
        self.goal = None
        self.plan_requested = False

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value,
                                 self.on_map, map_qos)
        self.create_subscription(PoseStamped, self.get_parameter("goal_topic").value,
                                 self.on_goal, 10)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value,
                                 self.on_odom, 20)
        plan_qos = QoSProfile(
            depth=2,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.plan_pub = self.create_publisher(
            Path, self.get_parameter("plan_topic").value, plan_qos)
        self.info_pub = self.create_publisher(String, self.get_parameter("info_topic").value, 10)
        self.create_timer(1.0, self.try_plan)
        self.get_logger().info("A* 规划节点已启动，等待 /map、/odom 与 /goal_pose")

    def on_map(self, msg):
        self.grid = grid_from_occupancy(msg.data, msg.info.width, msg.info.height)
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        radius = float(self.get_parameter("robot_radius").value)
        extra = float(self.get_parameter("inflation_extra").value)
        radius_cells = math.ceil((radius + extra) / self.resolution)
        self.inflated = inflate_grid(self.grid, radius_cells)
        self.get_logger().info(
            "收到地图 %dx%d，膨胀 %d 格（机器人半径 %.2f m）"
            % (self.grid.shape[1], self.grid.shape[0], radius_cells, radius))

    def on_odom(self, msg):
        pose = msg.pose.pose
        self.start_pose = (pose.position.x, pose.position.y,
                           yaw_from_quaternion(pose.orientation))

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.plan_requested = True
        self.get_logger().info("收到目标点 (%.2f, %.2f)" % self.goal)

    def try_plan(self):
        if self.inflated is None or self.start_pose is None:
            return
        if self.goal is None:
            gx = float(self.get_parameter("auto_goal_x").value)
            gy = float(self.get_parameter("auto_goal_y").value)
            if math.isnan(gx) or math.isnan(gy):
                return
            self.goal = (gx, gy)
            self.plan_requested = True
        if not self.plan_requested:
            return
        self.plan_requested = False
        self.plan()

    def _cell(self, x, y):
        col = int((x - self.origin[0]) / self.resolution)
        row = int((y - self.origin[1]) / self.resolution)
        return row, col

    def plan(self):
        started = time.time()
        start = self._cell(self.start_pose[0], self.start_pose[1])
        goal = self._cell(self.goal[0], self.goal[1])
        path, info = find_path(
            self.inflated, start, goal,
            allow_diagonal=bool(self.get_parameter("allow_diagonal").value),
            weight=float(self.get_parameter("weight").value),
            max_expansions=int(self.get_parameter("max_expansions").value),
        )
        elapsed_ms = (time.time() - started) * 1000.0
        info.update({
            "start_world": [self.start_pose[0], self.start_pose[1]],
            "goal_world": list(self.goal),
            "start_cell": list(start),
            "goal_cell": list(goal),
            "cells": len(path),
            "elapsed_ms": round(elapsed_ms, 2),
            "robot_radius": float(self.get_parameter("robot_radius").value),
            "inflation_extra": float(self.get_parameter("inflation_extra").value),
        })

        if not path:
            self.get_logger().error("规划失败：%s" % info["reason"])
            self.info_pub.publish(String(data=json.dumps(info, ensure_ascii=False)))
            return

        world_points = []
        for row, col in path:
            wx = self.origin[0] + (col + 0.5) * self.resolution
            wy = self.origin[1] + (row + 0.5) * self.resolution
            world_points.append((wx, wy))
        length = path_length_world(world_points)
        info["cost_cells"] = round(info["cost"], 3)
        info["length_m"] = round(length, 3)

        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for i, (wx, wy) in enumerate(world_points):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            if i + 1 < len(world_points):
                yaw = math.atan2(world_points[i + 1][1] - wy,
                                 world_points[i + 1][0] - wx)
            elif world_points:
                yaw = math.atan2(wy - world_points[i - 1][1],
                                 wx - world_points[i - 1][0])
            else:
                yaw = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(pose)
        self.plan_pub.publish(msg)
        self.info_pub.publish(String(data=json.dumps(info, ensure_ascii=False)))
        self.get_logger().info(
            "规划成功：%d 格 / %.2f m，扩展 %d 节点，用时 %.1f ms"
            % (len(path), length, info["expanded"], elapsed_ms))


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
