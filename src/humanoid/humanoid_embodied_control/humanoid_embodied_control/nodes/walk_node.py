# -*- coding: utf-8 -*-
"""ROS 2 Humble 节点：把控制内核接到 rclpy。

话题约定（与仓库其它模块保持一致）：

    订阅  /uav/lidar          sensor_msgs/LaserScan       环境感知
    订阅  /goal               geometry_msgs/PointStamped  目标点
    发布  /uav/cmd_vel        geometry_msgs/Twist         速度/转向指令载体
    发布  /uav/state          std_msgs/String             JSON 状态遥测

设计：控制内核 :class:`WalkNodeCore` 与 ROS 无关，本文件只做 rclpy 胶水
（订阅回调、定时器、消息收发），单测不需要 ROS 环境。
"""

from __future__ import annotations

import json
import math


class WalkNodeCore:
    """与 ROS 无关的节点内核：输入感知/目标，输出控制指令与遥测。

    这样拆开的原因：rclpy 的订阅/定时器 API 与纯控制逻辑解耦，控制逻辑
    写成纯 Python 后，单测不需要 ROS 环境，ROS 适配层只有几十行。
    """

    def __init__(self, controller=None):
        from humanoid_embodied_control.controller import WalkingController
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
        self.cmd = self.ctrl.balance.compute_balance(
            quat, gyro, (x, y, 0.95), (0.0, 0.0, 0.0))
        self.telemetry["state"] = state.value
        self.telemetry["speed"] = round(speed, 3)
        return speed, turn, target

    def telemetry_json(self) -> str:
        return json.dumps(self.telemetry, ensure_ascii=False)


def main():
    """rclpy 节点入口（``ros2 run humanoid_embodied_control humanoid_walk_node``）。"""
    try:
        import rclpy
        from geometry_msgs.msg import PointStamped, Twist
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
    except ImportError as exc:
        raise SystemExit(
            "未找到 rclpy：请在 ROS 2 Humble 环境下运行 "
            "（source /opt/ros/humble/setup.bash）。原始错误：%s" % exc)

    from humanoid_embodied_control.controller import WalkingController

    rclpy.init()
    node = rclpy.create_node("humanoid_walk_node")

    goal_x = float(node.declare_parameter("goal_x", 4.5).value)
    goal_y = float(node.declare_parameter("goal_y", 0.0).value)
    stride = float(node.declare_parameter("stride", 0.20).value)
    lift = float(node.declare_parameter("lift", 0.10).value)
    period = float(node.declare_parameter("period", 1.3).value)
    ctrl_hz = float(node.declare_parameter("ctrl_hz", 100.0).value)

    controller = WalkingController(goal=(goal_x, goal_y), stride=stride,
                                   lift=lift, period=period)
    core = WalkNodeCore(controller=controller)

    pub_cmd = node.create_publisher(Twist, "/uav/cmd_vel", 1)
    pub_state = node.create_publisher(String, "/uav/state", 1)

    # 里程/IMU 话题接入前，先用零值占位；真实部署由对应回调刷新。
    state = {"quat": (1.0, 0.0, 0.0, 0.0), "gyro": (0.0, 0.0, 0.0),
             "front": 3.0, "yaw": 0.0, "x": 0.0, "y": 0.0}

    def on_scan(msg: LaserScan):
        state["front"] = core.on_scan(list(msg.ranges), msg.angle_min,
                                      msg.angle_increment, msg.range_max)

    def on_goal(msg: PointStamped):
        core.on_goal(msg.point.x, msg.point.y, msg.point.z)

    node.create_subscription(LaserScan, "/uav/lidar", on_scan, 1)
    node.create_subscription(PointStamped, "/goal", on_goal, 1)

    node.get_logger().info(
        "humanoid_walk_node 启动 goal=(%.2f, %.2f) period=%.2f stride=%.2f "
        "lift=%.2f ctrl=%.0fHz" % (goal_x, goal_y, period, stride, lift, ctrl_hz))

    dt = 1.0 / ctrl_hz
    t0 = node.get_clock().now().nanoseconds / 1e9

    def tick():
        now = node.get_clock().now().nanoseconds / 1e9
        t = now - t0
        speed, turn, target = core.on_tick(
            t, state["x"], state["y"], state["yaw"], state["front"],
            state["quat"], state["gyro"], dt)

        cmd = Twist()
        cmd.linear.x = core.cmd[0] if core.cmd else 0.0
        cmd.linear.y = core.cmd[1] if core.cmd else 0.0
        cmd.linear.z = core.cmd[2] if core.cmd else 0.0
        cmd.angular.x = speed
        cmd.angular.y = turn
        pub_cmd.publish(cmd)

        msg = String()
        msg.data = json.dumps(
            {"t": round(t, 3), "speed": round(speed, 3),
             "turn": round(turn, 3), "targets": target,
             **core.telemetry}, ensure_ascii=False)
        pub_state.publish(msg)

    node.create_timer(dt, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
