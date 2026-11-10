# -*- coding: utf-8 -*-
"""ROS 1 / ROS 2 通用节点骨架（ROS 1 Noetic 优先）。

本文件不 import 任何 ROS 符号，真正的 rclpy / rospy 胶水由
:mod:`humanoid_embodied_control.nodes.ros1_adapter` 提供。这样同一份
控制逻辑在 ROS 1 与 ROS 2 下都能跑，单测也不需要 ROS 环境。

话题约定（与仓库其它模块保持一致）：

    订阅  /uav/lidar          sensor_msgs/LaserScan       环境感知
    订阅  /uav/depth          sensor_msgs/Image           深度相机（可选）
    订阅  /goal               geometry_msgs/PointStamped  目标点
    发布  /uav/cmd_vel        geometry_msgs/Twist         关节/根部控制指令
    发布  /uav/state          std_msgs/String             JSON 状态遥测
"""

from __future__ import annotations

import json
import math


class WalkNodeCore:
    """与 ROS 无关的节点内核：输入感知/目标，输出控制指令与遥测。

    这样拆开的原因：ROS 1（rospy）与 ROS 2（rclpy）的订阅/定时器 API
    差别很大，但控制逻辑必须共享；写成纯 Python 内核后，单测不需要 ROS 环境，
    同时 ROS 1 / ROS 2 的适配层只有几十行。
    """

    def __init__(self, controller=None):
        # 延迟 import：纯函数层不依赖 mujoco / rospy
        from humanoid_embodied_control.scripts.main import WalkingController
        self.ctrl = controller or WalkingController()
        self.cmd = None
        self.telemetry = {}

    # ------------------------------------------------------------------
    def on_scan(self, ranges, angle_min, angle_increment, range_max):
        """收到一帧激光扫描 → 更新前向可通行距离。"""
        from humanoid_embodied_control import perception

        angle_span = angle_increment * len(ranges) if ranges else 0.0
        sectors = perception.lidar_sectors(list(ranges), angle_span, range_max, 16)
        front = perception.front_clearance(sectors, angle_span, range_max)
        self.telemetry["front"] = round(front, 3)
        self.telemetry["sectors"] = [round(s, 2) for s in sectors]
        return front

    def on_goal(self, x, y, z=0.0):
        """收到目标点。"""
        self.ctrl.goal = (float(x), float(y))
        self.telemetry["goal"] = [round(float(x), 3), round(float(y), 3)]

    def on_tick(self, t, x, y, yaw, front_clear, quat, gyro, dt):
        """控制周期：输出 (速度标量, 转向偏置, 12 维关节目标角)。"""
        goal_dist = math.hypot(self.ctrl.goal[0] - x, self.ctrl.goal[1] - y)
        speed, state, turn = self.ctrl.command(t, x, y, yaw, front_clear, goal_dist)
        target = self.ctrl.gait_target(dt, speed, turn, quat, gyro)
        self.cmd = self.ctrl.balance.compute_balance(quat, gyro, (x, y, 0.95), (0.0, 0.0, 0.0))
        self.telemetry["state"] = state.value
        self.telemetry["speed"] = round(speed, 3)
        return speed, turn, target

    def telemetry_json(self) -> str:
        return json.dumps(self.telemetry, ensure_ascii=False)


def main():
    """ROS 2 入口。ROS 1 下用的是 roslaunch 里的节点脚本，不经过这里。"""
    raise SystemExit(
        "ROS 2 入口请用：ros2 run humanoid_embodied_control humanoid_walk_node")


if __name__ == "__main__":
    main()
