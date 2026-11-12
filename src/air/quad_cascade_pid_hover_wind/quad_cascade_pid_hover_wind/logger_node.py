#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""数据记录节点：CSV 记录状态、控制量、目标、风速。"""

import csv
import json
import math
import os
import time
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class LoggerNode(Node):
    def __init__(self):
        super().__init__("quad_logger")
        self.declare_parameter("output_dir", "/tmp/quad_results")
        self.declare_parameter("log_rate", 50.0)
        self.declare_parameter("duration", 40.0)

        self.out_dir = self.get_parameter("output_dir").value
        os.makedirs(self.out_dir, exist_ok=True)
        self.log_rate = float(self.get_parameter("log_rate").value)
        self.duration = float(self.get_parameter("duration").value)

        self.odom: Optional[Odometry] = None
        self.gt: Optional[Odometry] = None
        self.wind: Optional[Vector3Stamped] = None
        self.cmd: Optional[Float32MultiArray] = None
        self.target: Optional[Point] = None

        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Odometry, "/ground_truth", self.on_gt, 10)
        self.create_subscription(Vector3Stamped, "/wind", self.on_wind, 10)
        self.create_subscription(Float32MultiArray, "/cmd_motors", self.on_cmd, 10)
        self.create_subscription(Point, "/target", self.on_target, 10)

        self.start_time = time.time()
        self.rows = []
        self.status = "running"
        self.final_pos = None

        self.timer = self.create_timer(1.0 / self.log_rate, self.log)
        self.get_logger().info("日志节点已启动：输出到 %s" % self.out_dir)

    def on_odom(self, m): self.odom = m
    def on_gt(self, m): self.gt = m
    def on_wind(self, m): self.wind = m
    def on_cmd(self, m): self.cmd = m
    def on_target(self, m): self.target = m

    def log(self):
        if self.odom is None:
            return
        t = time.time() - self.start_time
        o = self.odom
        p = o.pose.pose.position
        v = o.twist.twist.linear
        w = o.twist.twist.angular
        quat = o.pose.pose.orientation
        q = np.array([quat.w, quat.x, quat.y, quat.z])
        from .geometry import quat_to_euler
        roll, pitch, yaw = quat_to_euler(q)

        wind = [0.0, 0.0, 0.0]
        if self.wind is not None:
            wind = [self.wind.vector.x, self.wind.vector.y, self.wind.vector.z]
        cmd = [0.0, 0.0, 0.0, 0.0]
        if self.cmd is not None and len(self.cmd.data) >= 4:
            cmd = list(self.cmd.data[:4])
        tgt = [0.0, 0.0, 0.0]
        if self.target is not None:
            tgt = [self.target.x, self.target.y, self.target.z]

        self.rows.append([
            round(t, 4),
            p.x, p.y, p.z, v.x, v.y, v.z,
            roll, pitch, yaw, w.x, w.y, w.z,
            wind[0], wind[1], wind[2],
            cmd[0], cmd[1], cmd[2], cmd[3],
            tgt[0], tgt[1], tgt[2],
        ])
        self.final_pos = [p.x, p.y, p.z]

        if t >= self.duration:
            self.finish()

    def finish(self):
        # 幂等：t >= duration 时 log() 已调用一次，
        # KeyboardInterrupt 路径可能再次进入，避免重写输出文件
        if self.status == "finished":
            return
        header = ["time", "x", "y", "z", "vx", "vy", "vz",
                  "roll", "pitch", "yaw", "wx", "wy", "wz",
                  "wind_x", "wind_y", "wind_z",
                  "m1", "m2", "m3", "m4",
                  "tx", "ty", "tz"]
        path = os.path.join(self.out_dir, "trajectory.csv")
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.rows)

        self.status = "finished"
        info = {
            "duration_s": round(time.time() - self.start_time, 2),
            "samples": len(self.rows),
            "final_position": self.final_pos,
            "log_rate": self.log_rate,
            "status": self.status,
        }
        with open(os.path.join(self.out_dir, "run_info.json"), "w") as f:
            json.dump(info, f, indent=2, ensure_ascii=False)
        self.get_logger().info("记录完成：%d 条 -> %s" % (len(self.rows), path))
        # 写一个完成标志
        with open(os.path.join(self.out_dir, "status.json"), "w") as f:
            json.dump({"status": "finished", "samples": len(self.rows)}, f)
        self.destroy_timer(self.timer)


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
