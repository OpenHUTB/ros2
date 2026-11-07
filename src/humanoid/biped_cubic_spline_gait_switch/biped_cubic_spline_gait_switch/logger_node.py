#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验记录节点：把关节、基座、质心/ZMP 与步态信息写入 CSV。"""

import csv
import json
import math
import os
import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String

HEADER = [
    "time", "gait", "phase", "switching", "switch_event", "progress",
    "base_x", "base_v", "hip_z", "lean",
    "q0", "q1", "q2", "q3", "q4", "q5",
    "qd0", "qd1", "qd2", "qd3", "qd4", "qd5",
    "qdes0", "qdes1", "qdes2", "qdes3", "qdes4", "qdes5",
    "com_x", "com_z", "zmp_x", "margin",
    "lf_x", "lf_z", "rf_x", "rf_z", "lc", "rc",
]


class LoggerNode(Node):
    def __init__(self):
        super().__init__("biped_logger")
        self.declare_parameter("output_dir", "results/run_spline")
        self.declare_parameter("duration", 34.0)
        self.declare_parameter("mode", "spline")
        self.declare_parameter("rate", 100.0)

        self.output_dir = os.path.abspath(self.get_parameter("output_dir").value)
        os.makedirs(self.output_dir, exist_ok=True)
        self.duration = float(self.get_parameter("duration").value)
        self.mode = str(self.get_parameter("mode").value)
        self.rate = float(self.get_parameter("rate").value)

        self.q_actual = None
        self.qd_actual = None
        self.q_des = None
        self.base = None
        self.metrics = None
        self.info = {}
        self.samples = 0
        self.finished = False
        self.start_time = self.get_clock().now()

        self.csv_file = open(os.path.join(self.output_dir, "trajectory.csv"), "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(HEADER)

        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(JointState, "/joint_cmd", self.on_cmd, 10)
        self.create_subscription(Odometry, "/base_state", self.on_base, 10)
        self.create_subscription(Float32MultiArray, "/biped_metrics", self.on_metrics, 10)
        self.create_subscription(String, "/gait_info", self.on_info, 20)

        self.create_timer(1.0 / self.rate, self.write_row)
        self.get_logger().info("记录节点启动，输出到 %s" % self.output_dir)
        with open(os.path.join(self.output_dir, "run_info.json"), "w") as f:
            json.dump({"mode": self.mode, "duration": self.duration,
                       "rate": self.rate}, f, ensure_ascii=False, indent=2)

    def on_joints(self, msg: JointState):
        if len(msg.position) >= 6:
            self.q_actual = list(msg.position[:6])
            self.qd_actual = list(msg.velocity[:6]) if len(msg.velocity) >= 6 else [0.0] * 6

    def on_cmd(self, msg: JointState):
        if len(msg.position) >= 6:
            self.q_des = list(msg.position[:6])

    def on_base(self, msg: Odometry):
        self.base = msg

    def on_metrics(self, msg: Float32MultiArray):
        if len(msg.data) >= 13:
            self.metrics = list(msg.data)

    def on_info(self, msg: String):
        try:
            self.info = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def write_row(self):
        if self.finished or self.q_actual is None or self.base is None or self.metrics is None:
            return
        info = self.info
        elapsed = float(info.get("time", 0.0))
        lean_y = float(self.base.pose.pose.orientation.y)
        lean = 2.0 * math.asin(max(-1.0, min(1.0, lean_y)))
        row = [
            elapsed, info.get("gait", ""), info.get("phase", 0.0),
            1 if info.get("switching", False) else 0,
            1 if info.get("switch_event", False) else 0,
            info.get("progress", 0.0),
            self.metrics[10], info.get("base_v", 0.0),
            self.metrics[11], round(lean, 6),
        ]
        row += [round(v, 6) for v in self.q_actual]
        row += [round(v, 6) for v in (self.qd_actual or [0.0] * 6)]
        row += [round(v, 6) for v in (self.q_des or [0.0] * 6)]
        row += [self.metrics[0], self.metrics[1], self.metrics[2], self.metrics[12],
                self.metrics[4], self.metrics[5], self.metrics[6], self.metrics[7],
                int(self.metrics[8]), int(self.metrics[9])]
        self.writer.writerow(row)
        self.samples += 1

        if self.samples % 100 == 0:
            self.csv_file.flush()

        if elapsed >= self.duration:
            self.finish(elapsed)

    def finish(self, elapsed: float):
        if self.finished:
            return
        self.finished = True
        self.csv_file.flush()
        self.csv_file.close()
        with open(os.path.join(self.output_dir, "status.json"), "w") as f:
            json.dump({"status": "finished", "samples": self.samples,
                       "duration": elapsed, "mode": self.mode}, f,
                      ensure_ascii=False, indent=2)
        self.get_logger().info("记录完成：%d 个采样，%.2f s" % (self.samples, elapsed))
        # 在回调内直接调用 rclpy.shutdown() 可能阻塞，放到独立线程中执行
        threading.Thread(target=rclpy.shutdown, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.finished:
            node.csv_file.flush()
            node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
