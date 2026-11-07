#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基于里程计反馈的纯追踪控制器。

订阅 /plan 与 /odom，按固定频率发布 /cmd_vel；到达目标或超时后发布
/nav_status 并写出状态文件，供一键实验脚本判断运行结束。
"""

import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from diff_drive_astar_odom_nav.control import pure_pursuit_step


class TrackerNode(Node):
    def __init__(self):
        super().__init__("odom_tracker")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/nav_status")
        self.declare_parameter("control_rate", 30.0)
        self.declare_parameter("lookahead", 0.55)
        self.declare_parameter("v_max", 0.75)
        self.declare_parameter("v_min", 0.18)
        self.declare_parameter("w_max", 1.40)
        self.declare_parameter("goal_tolerance", 0.12)
        self.declare_parameter("max_time", 90.0)
        self.declare_parameter("status_file", "/tmp/diff_drive_astar_odom_nav_status.json")

        self.path = []
        self.pose = None
        self.last_index = 0
        self.running = False
        self.started = None
        self.last_progress_time = None
        self.best_distance = float("inf")
        self.status = "IDLE"
        self.finished = False

        plan_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Path, self.get_parameter("plan_topic").value,
                                 self.on_plan, plan_qos)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value,
                                 self.on_odom, 20)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_topic").value, 10)
        self.status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 10)
        rate = max(5.0, float(self.get_parameter("control_rate").value))
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info("里程计反馈跟踪器已启动，等待 /plan 与 /odom")

    def on_odom(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (pose.position.x, pose.position.y, yaw)

    def on_plan(self, msg):
        if not msg.poses:
            return
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.last_index = 0
        self.best_distance = float("inf")
        self.started = time.time()
        self.last_progress_time = self.started
        self.running = True
        self.finished = False
        self._set_status("RUNNING")
        self.get_logger().info("收到全局路径 %d 点，开始里程计闭环跟踪" % len(self.path))

    def _set_status(self, status):
        self.status = status
        self.status_pub.publish(String(data=status))

    def _write_status_file(self, extra):
        path = self.get_parameter("status_file").value
        if not path:
            return
        data = {"status": self.status, "path_points": len(self.path)}
        data.update(extra)
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            tmp = path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            os.replace(tmp, path)
        except OSError as exc:
            self.get_logger().warn("状态文件写入失败：%s" % exc)

    def _stop(self, status, extra=None):
        self.running = False
        self.finished = True
        self.cmd_pub.publish(Twist())
        self._set_status(status)
        payload = {"elapsed": round(time.time() - self.started, 2) if self.started else 0.0}
        if extra:
            payload.update(extra)
        self._write_status_file(payload)
        self.get_logger().info("导航结束：%s %s" % (status, payload))

    def on_timer(self):
        if not self.running or not self.path or self.pose is None:
            return
        now = time.time()
        step = pure_pursuit_step(
            self.path, self.pose, self.last_index,
            lookahead=float(self.get_parameter("lookahead").value),
            v_max=float(self.get_parameter("v_max").value),
            v_min=float(self.get_parameter("v_min").value),
            w_max=float(self.get_parameter("w_max").value),
            goal_tolerance=float(self.get_parameter("goal_tolerance").value),
        )
        self.last_index = max(self.last_index, step["nearest"])

        if step["reached"]:
            self._stop("SUCCEEDED", {
                "distance_to_goal": round(step["distance"], 3),
                "final_index": step["nearest"],
            })
            return

        if step["distance"] < self.best_distance - 0.01:
            self.best_distance = step["distance"]
            self.last_progress_time = now

        max_time = float(self.get_parameter("max_time").value)
        if now - self.started > max_time or now - self.last_progress_time > 15.0:
            reason = "timeout" if now - self.started > max_time else "stuck"
            self._stop("FAILED", {
                "reason": reason,
                "distance_to_goal": round(step["distance"], 3),
                "final_index": step["nearest"],
            })
            return

        cmd = Twist()
        cmd.linear.x = step["v"]
        cmd.angular.z = step["w"]
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.running:
            node._stop("ABORTED")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
